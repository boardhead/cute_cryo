//-----------------------------------------------------------------------------
//
// File:        cute_server.js
//
// Description: CUTE cryostat position control hardware interface
//
//              This is a TCP server running on a system that interfaces
//              multiple AVR32 boards attached via USB, and an ADAM-6017
//              8-channel ADC attached via TCP.  The AVR32 boards are used as
//              stepper-motor controllers and for digital i/o, and the ADAM is
//              used to read out various analogue sensors.
//
//              Clients interact with this server via the cute_cryo.html page.
//
// Revisions:   2017-03-16 - v0.01 P. Harvey created
//              2017-04-28 - v0.9 PH - Implemented control algorithm
//
// Syntax:      node cute_server.js
//
// Notes:       Requires additional node libraries to run.  Install them
//              by typing: "npm install usb"
//
const kServerVersion    = 'v0.9';

const kAtmel            = 0x03eb;   // Atmel USB manufacturer ID
const kEVK1101          = 0x2300;   // EVK-1101 USB device ID
const kPosHisLen        = 600;      // position history length
const kHardwarePollTime = 80;       // hardware polling time (ms)
const kMaxBadPolls      = 3;        // number of bad polls before deactivating
const kMotorSlow        = 100;      // slowest drive speed (steps/s)

const kNumLimit         = 6;        // number of limit switches to poll: "PA0-<kNumLimit-1>"
const kTopLimit         = 0;        // PA0 is a top limit (and PA2, PA4, ...)
const kBotLimit         = 1;        // PA1 is a bottom limit (and PA3, PA5, ...)
const kHitLimit         = 0;        // digital value if we hit the limit switch
const kNotLimit         = 1;        // digital value if limit switch is not activated

const kDamperForceConst = 0.5;      // damper force constant (kg/mm)
const kLoadNom          = 35;       // nominal damper load at nominal air pressure (kg)
const kLoadMax          = 50;       // maximum damper load (kg) (lab jack limit)
const kLoadMin          = 20;       // minimum damper load (kg)
const kLoadTol          = 5;        // damper load tolerance (kg)
const kPositionNom      = 1;        // nominal position of damper top (mm)
const kPositionTol      = 0.1;      // tolerance in damper position (mm)
const kMotorStepsPer_mm = 4302;     // number of motor steps per mm of stage travel
const kMotorTol         = 1;        // maximum error in motor position (mm)

const kAirPressureNom   = 1241;     // nominal mine air pressure (hPa)
const kVacuumArea       = 100;      // cross-section area of vacuum manifold (cm^2) (TEMPORARY)
const kGravity          = 9.81;     // acceleration due to gravity (kg.m/s^2)

// states for ADAM-6017 (adamState)
const kAdamBad          = -1;       // communication error
const kAdamNotConnected = 0;        // Adam is not connected
const kAdamMissed       = 1;        // missed a response from the Adam
const kAdamWaiting      = 2;        // waiting for a response from the Adam
const kAdamOK           = 3;        // Adam responded OK

const kBanner = '---- CUTE Cryostat Position Control ' + kServerVersion + ' ----';

var avrSN = [
    'ffffffff3850313339302020ff1011',   // AVR0
    'ffffffff3850313339302020ff0d0b'    // AVR1
];

var adamIP = "130.15.24.85";    // ADAM-6017 IP
var adamPort = 502;             // ADAM-6017 port number

// IP's that are allowed to connect to the server
var authorized = {
    '*' : 1, // (uncomment this to allow commands from any IP)
    'localhost' : 1,
    '130.15.24.88' : 1
};

// online help (HTML format)
var helpMessage =
    'C <table width="100%" class=tbl>' +
    '<tr><td colspan=4>CUTE Commands:</th></tr>' +
    '<tr><td class=nr>/active [on|off]</td><td>- get/set active control</td>' +
        '<td class=nr>/log MSG</td><td>- enter log message</td></tr>' +
    '<tr><td class=nr>/avr# CMD</td><td>- send AVR command</td>' +
        '<td class=nr>/name [WHO]</td><td>- get/set client name</td></tr>' +
    '<tr><td class=nr>/cal</td><td>- show Adam calibration</td>' +
        '<td class=nr>/verbose [on|off]</td><td>- get/set verbose state</td></tr>' +
    "<tr><td class=nr>/list</td><td>- list connected AVR's</td>" +
        '<td class=nr>/who</td><td>- list connected clients</td></tr>' +
    '</table>';

var adam;               // client for communicating with ADAM-6017
var adamRaw = [ ];      // raw Adam ADC values (x8)
var adamState = kAdamNotConnected;

var usb = require('usb');
var fs = require('fs');
var WebSocketServer = require('websocket').server;
var http = require('http');

var avrs = [];          // AVR devices
var avrOK = [];         // flag set if AVR is responding OK
var foundAVRs = 0;      // number of recognized AVRs
var conn = [];          // web client connections
var active = 0;         // flag set if position control is active
var badPolls = 0;       // number of bad polls when active

var flashPin = ['pa07','pa08','pa21','pa08'];   // sequence to flash LED's
var flashNum = flashPin.length - 1;
var flashTime = 0;

var vals = [0,0,0];         // most recent measured values
var motorSpd = [0,0,0];     // motor speeds
var motorDir = [0,0,0];     // current motor directions
var limitSwitch = [0,0,0,0,0,0]; // limit-switches: top,bottom for each stage (1=open)
var lastSpd = '0 0 0';      // last motor speeds sent to clients (as string)
var motorPos = [0,0,0];     // motor positions
var history = [];           // history of measured values
var historyTime = -1;       // time of most recent history entry

var intrvl;                 // interval timer for polling hardware
var fullPoll = 0;
var verbose = 0;

var linear = [0, 0, 0, 0, 0, 0];    // linear transducer measurements (mm)
var damperPosition = [-1,-1,-1];    // positions of the 3 dampers (mm)
var stagePosition = [-1,-1,-1];     // current position of top of jack stand (mm)
var damperLoad = [0, 0, 0];         // current damper loads (kg)
var damperAddWeight = [0, 0, 0];    // weights to add to the 3 dampers (kg)
var airPressure = kAirPressureNom;  // measured air pressure (hPa)

//-----------------------------------------------------------------------------
// Main script

// create HTTP server (don't implement request listener here
// because that will be done by our WebSocketServer)
var server = http.createServer(function(request, response) { });
server.on('error', function(error) { Log('Server error!') });
server.listen(8080, function() { });

// create the server
wsServer = new WebSocketServer({ httpServer: server });

// handle connection requests from our web clients
wsServer.on('request', HandleClientRequest);

process.on('SIGINT', HandleSigInt); // handle CTRL-C (SIGINT signal)

usb.on('attach', OpenAVR);      // handle new USB devices being connected
usb.on('detach', ForgetAVR);    // handle USB devices being disconnected
usb.on('error', HandleError);   // handle USB errors

Log();              // (blank log line)
Log(kBanner);       // print startup banner

ConnectToAdam();    // connect to ADAM-6017 ADC via TCP
FindAVRs();         // find all connected AVR USB devices

// set interval timer to poll the hardware
intrvl = setInterval(PollHardware, kHardwarePollTime);

//-----------------------------------------------------------------------------
// poll the hardware
function PollHardware()
{
    var bad;

    fullPoll = !fullPoll;

    if (adamState != kAdamOK) {
        // send empty ADC readings if Adam didn't respond
        if (fullPoll) {
            var t = AddToHistory(0);
            PushData('F ' + (t % kPosHisLen));
        }
        if (!adam) ConnectToAdam();
        bad = 'Adam';
    } else if (!avrOK[0]) {
        bad = 'AVR0';
    }

    if (bad) {
        if (badPolls < kMaxBadPolls) ++badPolls;
        if (active && badPolls >= kMaxBadPolls) {
            Log(bad, "poll error!  Position control deactivated");
            Deactivate();
        }
    } else {
        badPolls = 0;
    }

    // poll ADAM-6017
    if (adamState == kAdamWaiting || adamState == kAdamOK) {
        // send command to read all 8 ADC's
        // 0x01, 0x00 = transaction ID = 0
        // 0x00, 0x00 = protocol ID = 0
        // 0x00, 0x06 = message length = 6 bytes to follow
        // 0x01       = unit ID = 1
        // 0x04       = function code = 4 (read input registers)
        // 0x00, 0x00 = address of first register = 0 (+ 0x4000)
        // 0x00, 0x08 = number of registers to read = 8 (16 bytes)
        if (!adam.write('010000000006010400000008', 'hex')) {
            Log("Adam write error");
        }
        if (adamState == kAdamOK) {
            adamState = kAdamWaiting;
        } else if (adamState == kAdamWaiting) {
            adamState = kAdamMissed;
            Log("Adam not responding");
        }
    } else if (adamState == kAdamNotConnected) {
        Log("Adam not connected!");
        adamState = kAdamBad;
    }

    for (var i=0; i<avrs.length; ++i) {
        if (avrs[i] == null) continue;
        avrOK[i] = 0;
        var cmd;
        switch (i) {
            case 0: // AVR0
                cmd = "f.m0;m1;m2;g.pa0-" + (kNumLimit-1) + "\n";   // poll limit switches
                break;
            case 1: // AVR1
                cmd = "c.nop\n";    // (nothing to do yet)
                break;
            default:
                // re-send "ser" command in case it was missed
                // (send even if we already got the "ser" response
                // because that should have triggered a "z" which
                // would need to be re-sent if we arrived here)
                cmd = "a.ser;b.ver\n";
                break;
        }
        avrs[i].SendCmd(cmd);
    }
}

//-----------------------------------------------------------------------------
// Calculate damper positions, loads, etc
function Calculate()
{
    for (var i=0; i<6; ++i) {
        // calculate linear distances (TEMPORARY)
        linear[i] = (35316 - adamRaw[i]) * (16.17 - 13.87) / (39856 - 35316) + 0;
    }
    // calculate air pressure (TEMPORARY)
    airPressure = kAirPressureNom + adamRaw[6] / 10;

    for (var i=0; i<3; ++i) {
        damperPosition[i] = linear[i];  // (TEMPORARY)
        // calculate load on this damper (TEMPORARY)
        stagePosition[i] = linear[i+3];
        damperLoad[i] = kLoadNom + (stagePosition[i] - linear[i]) * kDamperForceConst;
        // calculate nominal load on this damper (factor of 3 is for 3 dampers)
        var loadNom = kLoadNom - (airPressure - kAirPressureNom) * kVacuumArea / (300 * kGravity);
        // this is how much weight we need to add to achieve the nominal load
        damperAddWeight[i] = loadNom - damperLoad[i];
    }
}

//-----------------------------------------------------------------------------
// Drive motors for active position control
function Drive()
{
    for (var i=0; i<3; ++i) {
        // verify that motor position agrees with measured stage position
        if (Math.abs(motorPos[i] / kMotorStepsPer_mm - stagePosition[i]) > kMotorTol) {
            Log("Motor " + i + " error!  Position control deactivated");
            Deactivate();
            return;
        }
        var drive = 0; // (+1 = drive up, -1 = drive down)
        var pos = damperPosition[i];
        var load = damperLoad[i];
        if (load > kLoadMax) {
            // damper is over-loaded -- drive motor down
            drive = -1;
        } else if (load < kLoadMin) {
            // damper is under-loaded -- drive motor up
            drive = 1;
        } else if (pos < kPositionNom - kPositionTol && load < kLoadMax - kLoadTol) {
            // damper position is low -- drive motor up
            drive = 1;
        } else if (pos > kPositionNom + kPositionTol && load > kLoadMin + kLoadTol) {
            // damper position is high -- drive motor down
            drive = -1;
        } else if (motorSpd[i] > 0) {
            // stop if we have reached our goal or damper is near maximum
            if (pos >= kPositionNom || load > kLoadMax - kLoadTol) {
                RampMotor(i, 0);    // stop motor
            } else {
                drive = 1;          // continue driving;
            }
        } else if (motorSpd[i] < 0) {
            // stop if we have reached our goal or damper is near minimum
            if (pos <= kPositionNom || load < kLoadMin + kLoadTol) {
                RampMotor(i, 0);    // stop motor
            } else {
                drive = -1;         // continue driving
            }
        }
        // don't attempt to drive past limit of lab jack
        if ((drive > 0 && limitSwitch[i*2 + kTopLimit] == kNotLimit) ||
            (drive < 0 && limitSwitch[i*2 + kBotLimit] == kNotLimit))
        {
            // drive faster if we are far away from our destination
            var away = Math.abs(pos - kPositionNom);
            if (away > kPositionTol * 4) {
                drive *= 10;
            } else if (away > kPositionTol) {
                drive *= 2;
            }
            // drive the motor in the specified direction
            RampMotor(i, drive * kMotorSlow);
        }
    }
}

//=============================================================================
// WebSocket server communication

//-----------------------------------------------------------------------------
// handle requests from our web clients
function HandleClientRequest(request)
{
    try {
        var connection = request.accept("cute", request.origin);

        // add new connection to our list
        conn[conn.length] = connection;
        connection.cuteName = GetAddr(request);

        // define a few member functions
        connection.Activate = Activate;
        connection.SendData = function(str) { this.send(str, function ack(error) { }) };
        connection.Respond = function() {
            this.SendData('C <span class=res>'+EscapeHTML(Array.from(arguments).join(' '))+'</span><br/>');
        };
        connection.Log = function() {
            Log('['+this.cuteName+'] '+Array.from(arguments).join(' '));
        };
        connection.HandleServerCommand = HandleServerCommand;

        // handle all messages from users here
        connection.on('message', function(message) { this.HandleServerCommand(message) });

        connection.on('close', function(reason) {
            // close user connection
            for (var i=0; i<conn.length; ++i) {
                if (conn[i] == this) {
                    this.Log('Closed connection');
                    conn.splice(i,1); // remove from list
                    break;
                }
            }
        });

        // send opening message to our client
        connection.Respond(kBanner);
        connection.Respond('(' + foundAVRs, 'AVRs connected)');

        connection.Log('Connected');    // log this connection

        // send message indicating whether or not we are active
        connection.SendData('D ' + active);
        // send current motor state
        connection.SendData('E ' + lastSpd);

        // send measurement history (packet "B")
        for (var i=history.length-1; i>=0; --i) {
            connection.SendData('B '+((historyTime - i) % kPosHisLen)+' '+
                         history[i][0].toFixed(4)+' '+
                         history[i][1].toFixed(4)+' '+
                         history[i][2].toFixed(4));
        }
    }
    catch (err) {
        Log('Error handling client request');
    }
}

//-----------------------------------------------------------------------------
// Handle commands sent from our clients
// Inputs: this=WebSocketServer, message=message
function HandleServerCommand(message)
{
    var str;
    // process WebSocket message
    if (!this.cuteName) {
        Log('Message from unknown client!');
        return;
    }
    if (message.type === 'utf8') {
        if (!authorized[GetAddr(this)] && !authorized['*']) {
            this.Respond('Sorry, you are not authorized to issue commands');
            return;
        }
        // separate command from its optional argument (command ends with ":")
        var i = message.utf8Data.indexOf(':');
        var cmd;
        if (i > 0) {
            cmd = message.utf8Data.substr(0,i).trim().toLowerCase();
            str = message.utf8Data.substr(i+1).trim();
        } else {
            cmd = message.utf8Data.trim().toLowerCase();
            str = '';
        }
        // process the command
        switch (cmd) {

            case 'help':
                this.SendData(helpMessage);
                break;

            case 'active':
                this.Activate(str);
                break;

            case 'cal':
                if (adamState != kAdamOK) {
                    this.Respond('Adam is not OK');
                } else {
                    Log('Raw:', adamRaw.join(' '));
                    Log('Cal:', linear.map(function(x) { return x.toFixed(2) } ).join(' '), 
                        airPressure.toFixed(1));
                    Log('Loads:', damperLoad.map(function(x) { return x.toFixed(2) }).join(' '));
                }
                break;

            case 'list': {
                var num = 0;
                for (var i=0; i<avrs.length; ++i) if (avrs[i]) ++num;
                this.Respond(num, 'AVRs connected:');
                for (var i=0; i<avrs.length; ++i) {
                    if (avrs[i]) this.Respond('AVR'+i, avrs[i].avrSN);
                }
            }   break;

            case 'log':
                this.Log(str);
                break;

            case 'name':
                if (str.length) {
                    this.Log('/'+cmd, str);
                    this.cuteName = str;
                } else {
                    this.Respond('Your name is "'+this.cuteName+'"');
                }
                break;

            case 'verbose':
                if (str.length) {
                    if (str.toLowerCase() == 'on') {
                        verbose = 1;
                    } else {
                        verbose = 0;
                    }
                    this.Log('Verbose', verbose ? 'on' : 'off');
                } else {
                    this.Respond('Verbose', verbose ? 'on' : 'off');
                }
                break;

            case 'who':
                this.Respond('Current users:',
                    conn.map(function(e){
                        return e.cuteName + (e==this?' (you)':'')
                    },this).join(', '));
                break;

            default:
                // check for AVR commands
                if (cmd.length==4 && cmd.substr(0,3) == 'avr') {
                    var n = cmd.substr(3,1);
                    if (!isNaN(n)) {
                        this.Log('/'+cmd, str);
                        if (avrs[n]) {
                            avrs[n].SendCmd('e.'+str.trim().toLowerCase()+'\n');
                        } else {
                            this.Log('AVR'+n,'is not connected');
                        }
                        break;
                    }
                }
                this.Respond('Unknown command:', cmd);
                break;
        }
    } else if (message.type === 'binary') {
        // handle binary data here (message.binaryData)
        this.Respond("Received binary data length=" + message.binaryData.length);
    } else {
        this.Respond('Received unknown message type=' + message.type);
    }
}

//-----------------------------------------------------------------------------
// Get client address
// Inputs: WebSocketServer
function GetAddr(sock)
{
    var addr = sock.remoteAddress.replace('::ffff:','');
    if (addr == '::1') addr = 'localhost';
    return addr;
}

//-----------------------------------------------------------------------------
// Activate/deactivate position control with messages
function Activate(arg)
{
    var on = { 'off':0, '0':0, 'on':1, '1':1 }[arg.toLowerCase()];
    if (arg == '' || on == active) {
        this.Respond('Active control is', (on==null ? 'currently' : 'already'),
            (active=='1' ? 'on' : 'off'));
    } else if (on == '1') {
        if (avrs[0]) {
            active = 1;
            PushData('D 1');
            // turn on motors and set zero position to stagePosition = 0
            for (var i=0; i<3; ++i) {
                var pos = Math.floor(stagePosition[i] * kMotorStepsPer_mm);
                avrs[0].SendCmd('c.m' + i + ' on 1;m' + i + ' pos ' + pos + '\n');
                motorPos[i] = pos;
            }
            this.Log('Position control activated');
        } else {
            this.Log('Can not activate because AVR0 is not attached');
        }
    } else if (on == '0') {
        Deactivate();
        this.Log('Position control deactivated');
    } else {
        this.Respond('Invalid argument for "active" command');
    }
}

//-----------------------------------------------------------------------------
// Deactivate position control and stop motors if necessary (no messages)
function Deactivate()
{
    active = 0;
    if (avrs[0]) avrs[0].SendCmd("c.halt\n");
    PushData('D 0');
}

//-----------------------------------------------------------------------------
// Push data to all web clients
function PushData(str)
{
    for (var i=0; i<conn.length; ++i) {
        conn[i].SendData(str);
    }
}

//=============================================================================
// ADAM-6017 communication

//-----------------------------------------------------------------------------
// Connect to ADAM-6017
function ConnectToAdam()
{
    var net = require('net');
    adam = new net.Socket();

    // handle connect message
    adam.connect(adamPort, adamIP, function() {
        adamState = kAdamOK;
        Log('Adam attached');
    });
    
    // handle incoming data from the ADAM-6017
    adam.on('data', function(data) {
        if (data.length != 25) return;
        if (adamState == kAdamMissed) Log("Adam OK");
        adamState = kAdamOK;
        // read the returned values
        for (var i=0, j=9; i<8; ++i, j+=2) {
            adamRaw[i] = data[j] * 256 + data[j+1];
        }
        if (verbose) Log("Adam values:", adamRaw.join(' '));
        
        Calculate();            // calculate damper positions, loads, etc
        if (active) Drive();    // drive motors if active control is on

        if (!fullPoll) return;

        // save in history and send data back to web clients
        var t = AddToHistory(0, damperPosition);
        PushData('F ' + (t % kPosHisLen) + ' ' +
                 damperPosition.map( function(x) { return x.toFixed(4) } ).join(' ') + ' ' +
                 damperAddWeight.map( function(x) { return x.toFixed(4) } ).join(' ') + ' ' +
                 airPressure.toFixed(1));

        FlashLEDs(t);   // flashy lights
    });

    // handle connection close
    adam.on('close', CloseAdam);

    // handle communication errors
    adam.on('error', function() {
        if (adamState == kAdamNotConnected) Log("Adam communication error");
        CloseAdam();
    });
}

//-----------------------------------------------------------------------------
// End communication with Adam
function CloseAdam()
{
    if (adam) {
        adam.destroy();
        adam = null;
    }
    adamState = kAdamBad;
}

//=============================================================================
// AVR32 USB communication

//-----------------------------------------------------------------------------
// Enumerate USB devices to locate our AVR boards
function FindAVRs()
{
    var devs = usb.getDeviceList();
    for (var i=0; i<devs.length; ++i) {
        OpenAVR(devs[i]);
    }
}

//-----------------------------------------------------------------------------
// Open communication with our AVR device
function OpenAVR(device)
{
    if (device.deviceDescriptor.idVendor  != kAtmel ||
        device.deviceDescriptor.idProduct != kEVK1101)
    {
        return; // (not our device)
    }

    try {
        device.open();
        device.interfaces[0].claim();
        // find first unused input >= 2
        for (var i=2; ; ++i) {
            if (i < avrs.length && avrs[i]) continue;
            avrs[i] = device;
            device.avrNum = i;
            var endIn  = device.interfaces[0].endpoints[0];
            var endOut = device.interfaces[0].endpoints[1];
            endIn.avrNum  = i;
            endOut.avrNum = i;
            endIn.transferType  = usb.LIBUSB_TRANSFER_TYPE_BULK;
            endOut.transferType = usb.LIBUSB_TRANSFER_TYPE_BULK;
            endIn.timeout  = 1000;
            endOut.timeout = 1000;
            // (must install handlers before we start polling)
            endIn.on('data', HandleData);
            endIn.on('error', HandleError);
            endOut.on('error', HandleError);
            endIn.startPoll(4, 256);

            // add member function to send command to AVR
            device.SendCmd = SendCmd;
            // send initial command to get AVR serial number and software version
            device.SendCmd("a.ser;b.ver\n");
            break;
        }
    }
    catch (err) {
        Log('Error opening AVR device');
    }
}

//-----------------------------------------------------------------------------
// Send command to our AVR
// Note: Command must be prefixed by a response ID (eg. "c." to ignore response)
function SendCmd(cmd)
{
    try {
        this.interfaces[0].endpoints[1].transfer(cmd, function(error) {
            if (error) {
                Log('AVR'+this.avrNum, 'send error');
                ForgetAVR(this);
            }
        });
    }
    catch (err) {
        Log('AVR'+this.avrNum, 'send exception');
        ForgetAVR(this);
    }
}

//-----------------------------------------------------------------------------
// Forget about this AVR
function ForgetAVR(deviceOrEndpoint)
{
    var avrNum = deviceOrEndpoint.avrNum;
    if (avrNum == null) {
        Log('Unknown AVR detached!');
    } else if (avrs[avrNum]) {
        Log('AVR'+avrNum, 'detached!');
        if (avrNum < 2) --foundAVRs;
        avrs[avrNum] = null;
        avrOK[avrNum] = 0;
    }
}

//-----------------------------------------------------------------------------
// Handle USB errors
function HandleError(err)
{
    // Log("AVR error!");
}

//-----------------------------------------------------------------------------
// Handle data received from our AVR devices
function HandleData(data)
{
    var avrNum = this.avrNum;
    if (avrNum == null) {
        Log('Data from unknown device!');
        return;
    }
    var str = data.toString();
    // wait for null at end of string
    var j = str.indexOf('\0');
    if (j > -1) str = str.substr(0, j); // remove null
    // process each response separately
    var lines = str.split("\n");
    var id;
    for (j=0; j<lines.length; ++j) {
        var str = lines[j];
        if (!str.length) continue;
        if (str.length >= 4 && str.substr(1,1) == '.') {
            id = str.substr(0,1);
            str = str.substr(2);
        }
        if (id != 'e') {
            if (str.substr(0,2) != 'OK') {
                Log('AVR'+avrNum, 'Bad response:', str);
                continue;
            }
            str = str.substr(3);
        }
        // ignore truncated responses (may happen at startup if commands
        // were sent before AVR was fully initialized)
        if (!id) continue;
        avrOK[avrNum] = 1;
        avrNum = HandleResponse(avrNum, id, str);
    }
}

//-----------------------------------------------------------------------------
// Handle response from an AVR command
// Returns: new AVR number (may change in response to a "ser" command)
function HandleResponse(avrNum, responseID, msg)
{
    switch (responseID) {
        case 'a':   // a = get serial number
            var avr;
            avrs[avrNum].avrSN = msg;   // save s/n in device object
            for (var i=0; i<avrSN.length; ++i) {
                if (msg == avrSN[i]) {
                    avr = 'AVR' + i;
                    if (avrNum < 2 || !avrs[avrNum]) {
                        if (avrs[i] != avrs[avrNum]) {
                            Log(avr, 'INTERNAL ERROR');
                        } else {
                            // could get here if we got a "ser" response
                            // from an already identified AVR
                            return avrNum; // (nothing to do)
                        }
                    } else {
                        if (avrs[i]) {
                            if (avrs[i] != avrs[avrNum]) {
                                Log(avr, 'ERROR! Already exists!');
                            }
                        } else {
                            ++foundAVRs;
                        }
                        // change the number of this AVR
                        avrs[i] = avrs[avrNum];
                        avrs[i].avrNum = i;
                        avrs[i].interfaces[0].endpoints[0].avrNum = i;
                        avrs[i].interfaces[0].endpoints[1].avrNum = i;
                        avrs[avrNum] = null;
                        avrOK[i] = 1;
                        avrOK[avrNum] = 0;
                        avrNum = i;
                    }
                    break;
                }
            }
            if (avr) {
                // enable watchdog timer
                avrs[avrNum].SendCmd('c.wdt 1\n');
                if (avrNum == 0) {
                    // enable pull-ups for limit switches
                    avrs[avrNum].SendCmd('c.pa0-' + (kNumLimit-1) + ' ' +
                        Array(kNumLimit+1).join('+') + '\n');
                    // set polarity of motor on sigals
                    avrs[avrNum].SendCmd('c.m0 on +;m1 on +;m2 on +\n');
                    // turn on motors
                    avrs[avrNum].SendCmd('c.m0 on 1;m1 on 1;m2 on 1\n');
                }
            } else {
                avr = 'Unknown AVR ' + avrNum;
                avrs[avrNum].SendCmd('z.wdt 0\n');  // disable watchdog on unknown AVR
            }
            Log(avr, 'attached (s/n', msg + ')');
            break;

        case 'b':   // b = log this response
            Log('AVR'+avrNum, msg);
            break;

        case 'c':   // c = ignore this response
            break;

        case 'd':   // d = handle ADC read responses
            var j = msg.indexOf('VAL=');
            // (currently not used)
            //if (j >= 0) {
            //    // ie. "adc2 512"
            //    var adcVal[msg.substr(3,1).Number()] = msg.substr(j+4).Number();
            //}
            break;

        case 'e':   // e = manual AVR command
            Log('[AVR'+avrNum+']', msg);
            break;

        case 'f': { // f = motor speeds
            if (msg.length >= 8 && msg.substr(0,1) == 'm') {
                var n = msg.substr(1,1);    // motor number
                var a = msg.split(' ');
                for (var i=0; i<a.length; ++i) {
                    switch (a[i].substr(0,4)) {
                        case "SPD=":
                            motorSpd[n] = Number(a[i].substr(4));
                            motorDir[n] = a[i].substr(4,1) == '-' ? 1 : 0;
                            break;
                        case "POS=":
                            motorPos[n] = Number(a[i].substr(4));
                            break;
                    }
                }
                if (n==2 && fullPoll) {
                    var newSpd = motorSpd.join(' ');
                    if (lastSpd != newSpd) {
                        PushData('E ' + newSpd);
                        lastSpd = newSpd;
                    }
                }
            }
        }   break;

        case 'g': { // g = poll limit switches
            var j = msg.indexOf('VAL=');
            if (j < 0 || msg.length - j < kNumLimit) {
                avrs[0].SendCmd("c.halt\n");
                Log("Poll error.  Motors halted");
                for (var k=0; k<kNumLimit; ++k) {
                    limitSwitch[k] = kHitLimit; // safety fallback: assume we hit the limit
                }
            } else {
                for (var k=0; k<kNumLimit; ++k) {
                    if (msg.substr(j+4+k, 1) == kNotLimit) {
                        limitSwitch[k] = kNotLimit;
                    } else {
                        limitSwitch[k] = kHitLimit;
                        var mot = Math.floor(k / 2);
                        if (!motorSpd[mot]) continue;
                        var isBottom = ((k & 0x01) == kBotLimit);
                        if (isBottom) {
                            // allow positive motor speed when at bottom limit
                            if (motorSpd[mot] > 0) continue;
                        } else {
                            // allow negative motor speed when at top limit
                            if (motorSpd[mot] < 0) continue;
                        }
                        avrs[0].SendCmd("c.m" + mot + " halt\n");
                        var which = isBottom ? "lower" : "upper";
                        Log("M" + mot + " halted! (hit " + which + " limit switch)");
                    }
                }
            }
        } break;

        case 'z':   // z = disable watchdog timer
            // forget about the unknown AVR
            avrs[avrNum] = null;
            break;

        default:
            Log('AVR'+avrNum, 'Unknown response:', msg);
            break;
    }
    return avrNum;
}

//-----------------------------------------------------------------------------
// Ramp motor to specified speed (+ve = up, -ve = down)
function RampMotor(n, spd)
{
    if (motorSpd[n] == spd) return;

    var changeDir = '';

    if (motorSpd[n] * spd < 0) {
        spd = 0;    // must stop motor first
    } else if (motorSpd[n] == 0 && spd != 0) {
        // change motor direction if necessary
        var dir = spd > 0 ? 0 : 1;
        if (dir != motorDir[n]) changeDir = 'c.m' + n + ' dir ' + dir + ';';
    }
    avrs[0].SendCmd(changeDir + 'c.m' + n + ' ramp ' + Math.abs(spd) + '\n');
}

//-----------------------------------------------------------------------------
// Flash AVR LED lights
// Inputs: t=time in seconds
function FlashLEDs(t)
{
    if (t == flashTime) return;     // don't flash twice in the same second
    var flashNew = (flashNum + 1) % flashPin.length;
    var cmd = "c."+flashPin[flashNum]+" 1;c."+
                   flashPin[flashNew]+" 0\n";
    for (var i=0; i<2; ++i) {
        if (avrs[i]) avrs[i].SendCmd(cmd);
    }
    flashNum = flashNew;
    flashTime = t;
}

//=============================================================================
// Other functions

//-----------------------------------------------------------------------------
// SIGINT handler
function HandleSigInt()
{
    console.log('');    // (for linefeed after CTRL-C)
    Log('Exiting on SIGINT');
    Cleanup();
    // exit after giving time to log last message
    setTimeout(function() { process.exit(); }, 10);
}

//-----------------------------------------------------------------------------
// Escape string for HTML
function EscapeHTML(str)
{
    var entityMap = {
        "&": "&amp;",
        "<": "&lt;",
        ">": "&gt;"
    };
    return str.replace(/[&<>]/g, function(s) { return entityMap[s]; });
}

//-----------------------------------------------------------------------------
// Log message to console and file
function Log()
{
    var msg;
    var d = new Date();
    if (arguments.length) {
        msg = d.getFullYear() + '-' + Pad2(d.getMonth()+1) + '-' + Pad2(d.getDate()) + ' ' +
                Pad2(d.getHours()) + ':' + Pad2(d.getMinutes()) + ':' +
                Pad2(d.getSeconds()) + ' ' + Array.from(arguments).join(' ');
    } else {
        msg = '';
    }
    console.log(msg);
    fs.appendFile('cute_server_'+d.getFullYear()+Pad2(d.getMonth()+1)+'.log', msg+'\n', 
        function(error) {
            if (error) console.log(error, 'writing log file');
        }
    );
    // send back to clients
    PushData('C ' +  EscapeHTML(msg) + '<br/>');
}

//-----------------------------------------------------------------------------
// Add to history of measurements
// Inputs: index/value pairs
// Returns: integer time of measurement
function AddToHistory(i,vals)
{
    var d = new Date();
    var t = Math.ceil(d.getTime()/1000);
    var entry;
    if (!history.length) {
        entry = history[0] = [];
        historyTime = t;
    } else {
        // add entries up to this time if necessary
        while (historyTime < t) {
            ++historyTime;
            history.unshift([]);
            if (history.length > kPosHisLen) history.pop();
        }
        entry = history[0];
    }
    if (vals) {
        for (var j=0; j<vals.length; ++j) {
            entry[j+i] = vals[j];
        }
    }
    return t;
}

//-----------------------------------------------------------------------------
// Pad integer to 2 digits
function Pad2(num)
{
    var str = num.toString();
    return((str.length < 2) ? '0'+str : str);
}

//-----------------------------------------------------------------------------
// Clean up before program termination
function Cleanup()
{
    clearInterval(intrvl);  // stop our polling

    for (var i=0; i<avrs.length; ++i) {
        if (!avrs[i]) continue;
        var avri = avrs[i];
        avrs[i] = null;
        if (i < 2) --foundAVRs;
        // release any still-open interfaces
        try {
            avri.interfaces[0].release(true, function(error) { });
            avri.close();
        }
        catch (err) {
        }
    }
    CloseAdam();
}

//-----------------------------------------------------------------------------
// end
