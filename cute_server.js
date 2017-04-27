//-----------------------------------------------------------------------------
//
// File:        cute_server.js
//
// Description: CUTE cryostat position control hardware interface
//
// Revisions:   2017-03-16 - v0.01 P. Harvey created
//
// Syntax:      node cute_server.js
//
// Notes:       Requires additional node libraries to run.  Install them
//              by typing: "npm install usb"
//
const kServerVersion    = 'v0.01';

const kAtmel            = 0x03eb;
const kEVK1101          = 0x2300;
const kPosHisLen        = 600;      // position history length
const kNumLimit         = 2;        // number of limit switches
const kHardwarePollTime = 80;       // hardware polling time (ms)
const kMaxBadPolls      = 3;        // number of bad polls before deactivating

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

var adam;               // client for communicating with ADAM-6017
var adamValue = [ ];    // Adam ADC values (x8)
var adamState = kAdamNotConnected;

var usb = require('usb');
var fs = require('fs');
var WebSocketServer = require('websocket').server;
var http = require('http');

var avrs = [];          // AVR devices
var avrOK = [];         // flag set if AVR is responding OK
var foundAVRs = 0;      // number of recognized AVRs
var conn = [];          // http client connections
var active = 0;         // flag set if position control is active
var badPolls = 0;       // number of bad polls when active

var flashPin = ['pa07','pa08','pa21','pa08'];   // sequence to flash LED's
var flashNum = flashPin.length - 1;
var flashTime = 0;

var motorSpd = [0,0,0];     // motor speeds
var limitSwitch = [0,0,0,0,0,0,0];   // limit-switch values (1=open)
var lastSpd = '0 0 0';      // last motor speeds sent to clients (as string)
var motorPos = [0,0,0];     // motor positions
var history = [];           // history of measured values
var historyTime = -1;       // time of most recent history entry

var intrvl;                 // interval timer for polling hardware
var fullPoll = 0;
var verbose = 0;

var linear = [0, 0, 0, 0, 0, 0];    // linear transducer measurements (mm)
var damperPosition = [-1,-1,-1];    // positions of the 3 dampers (mm)
var damperAddWeight = [0, 0, 0];    // weight to add to the 3 dampers (kg)
var airPressure = 1013;             // measured air pressure (hPa)

//-----------------------------------------------------------------------------
// Main script

var server = http.createServer(function(request, response) {
    // process HTTP request. Since we're writing just WebSockets server
    // we don't have to implement anything.
});
server.on('error', function(error) { Log('Server error!') });
server.listen(8080, function() { });

// create the server
wsServer = new WebSocketServer({ httpServer: server });

// handle connection requests from our web clients
wsServer.on('request', function(request) { HandleClientRequest(request) });

// handle CTRL-C and SIGINT signal
process.on('SIGINT', function() { HandleSigInt() });

// handle new USB devices being connected
usb.on('attach', function(device) {
    if (device.deviceDescriptor.idVendor  == kAtmel &&
        device.deviceDescriptor.idProduct == kEVK1101)
    {
        OpenAVR(device);
    }
});
usb.on('error', HandleError);

// handle USB devices that have disconnected
usb.on('detach', ForgetAVR);

Log();
Log(kBanner);

ConnectToAdam();
FindAVRs();     // find all connected AVR boards

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
            Log(bad, "poll error!  System deactivated");
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
        linear[i] = (adamValue[i] - 35316) * (16.17 - 13.87) / (39856 - 35316) + 0;
    }
    for (var i=0; i<3; ++i) {
        damperPosition[i] = linear[i];
    }
}

//-----------------------------------------------------------------------------
// Drive motors for active position control
function Drive()
{
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

        connection.Respond(kBanner);
        connection.Respond('(' + foundAVRs, 'AVRs connected)');

        connection.Log('Connected');

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
                this.SendData(
                    'C <table width="100%" class=tbl>' +
                    '<tr><td colspan=4>CUTE Commands:</th></tr>' +
                    '<tr><td class=nr>/active [on|off]</td><td>- get/set active control</td>' +
                        '<td class=nr>/log</td><td>- enter log message</td></tr>' +
                    '<tr><td class=nr>/avr# CMD</td><td>- send AVR command</td>' +
                        '<td class=nr>/name [WHO]</td><td>- get/set client name</td></tr>' +
                    '<tr><td class=nr>/cal</td><td>- show Adam calibration</td>' +
                        '<td class=nr>/verbose [on|off]</td><td>- get/set verbose state</td></tr>' +
                    "<tr><td class=nr>/list</td><td>- list connected AVR's</td>" +
                        '<td class=nr>/who</td><td>- list connected clients</td></tr>' +
                    '</table>'
                );
                break;
            case 'active':
                this.Activate(str);
                break;
            case 'cal':
                if (adamState != kAdamOK) {
                    this.Respond('Adam is not OK');
                } else {
                    Log('Raw:', adamValue.join(' '));
                    Log('Cal:', linear.map(function(x) { return x.toFixed(2) } ).join(' '), 
                        airPressure.toFixed(1));
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
// Push data to all http clients
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
        if (data.length == 25) {
            if (adamState == kAdamMissed) Log("Adam OK");
            adamState = kAdamOK;
            // read the returned values
            for (var i=0; i<8; ++i) {
                var j = i * 2 + 9;
                adamValue[i] = data[i*2+9] * 256 + data[j+1];
            }
            if (verbose) Log("Adam values:", adamValue.join(' '));
            
            Calculate();            // calculate damper positions, loads, etc
            if (active) Drive();    // drive motors if active control is on

            if (fullPoll) {
                // save in history and send data back to http clients
                var t = AddToHistory(0, damperPosition);
                PushData('F ' + (t % kPosHisLen) + ' ' +
                         damperPosition.map( function(x) { return x.toFixed(4) } ).join(' ') + ' ' +
                         damperAddWeight.map( function(x) { return x.toFixed(4) } ).join(' ') + ' ' +
                         airPressure.toFixed(1));
                // flash some LED's for the fun of it
                if (t != flashTime) {
                    var flashNew = (flashNum + 1) % flashPin.length;
                    var cmd = "c."+flashPin[flashNum]+" 1;c."+
                                   flashPin[flashNew]+" 0\n";
                    for (var i=0; i<2; ++i) {
                        if (avrs[i]) avrs[i].SendCmd(cmd);
                    }
                    flashNum = flashNew;
                    flashTime = t;
                }
            }
        }
    });

    // handle connection close
    adam.on('close', function() {
        adamState = kAdamBad;
        if (adam) {
            adam.destroy();
            adam = null;
        }
    });

    // handle communication errors
    adam.on('error', function() {
        if (adamState == kAdamNotConnected) {
            Log("Adam communication error");
            adamState = kAdamBad;
        }
        if (adam) {
            adam.destroy();
            adam = null;
        }
    });
}

//=============================================================================
// AVR32 USB communication

//-----------------------------------------------------------------------------
// Enumerate USB devices to locate our AVR boards
function FindAVRs()
{
    var devs = usb.getDeviceList();
    for (var i=0; i<devs.length; ++i) {
        if (devs[i].deviceDescriptor.idVendor  == kAtmel &&
            devs[i].deviceDescriptor.idProduct == kEVK1101)
        {
            OpenAVR(devs[i]);
        }
    }
}

//-----------------------------------------------------------------------------
// Open communication with our AVR device
function OpenAVR(device)
{
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
            endIn.startPoll(4, 256);
            endIn.on('data', HandleData);
            endIn.on('error', HandleError);
            endOut.on('error', HandleError);

            // add member function to send command to AVR
            device.SendCmd = function SendCmd(cmd) {
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
            };
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
    // if (err) Log(err);
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
            //  Log('AVR'+avrNum, 'Bad response:', str);
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
                // 1. enable watchdog timer
                // 2. seand set up avr
                avrs[avrNum].SendCmd('c.wdt 1;m0 on +;pa0-1 ++\n');
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
                avrs[0].SendCmd("halt\n");
                Log("Poll error.  Motors halted");
                for (var k=0; k<kNumLimit; ++k) {
                    limitSwitch[k] = 0;
                }
            } else {
                for (var k=0; k<kNumLimit; ++k) {
                    if (msg.substr(j+4+k, 1) == '1') {
                        limitSwitch[k] = 1;
                    } else {
                        limitSwitch[k] = 0;
                        var mot = Math.floor(k / 2);
                        if (!motorSpd[mot]) continue;
                        // odd-numbered switches are lower limits
                        if (k & 0x01) {
                            if (motorSpd[mot] > 0) continue;
                        } else {
                            if (motorSpd[mot] < 0) continue;
                        }
                        avrs[0].SendCmd("m" + mot + " halt\n");
                        Log("M" + mot + " halted! (hit " + (k & 0x01 ? "lower" : "upper") + " limit switch)");
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
// Activate/deactivate position control with messages
function Activate(arg)
{
    var on = { 'off':0, '0':0, 'on':1, '1':1 }[arg.toLowerCase()];
    if (arg == '' || on == active) {
        this.Respond('Active control is', (on==null ? 'currently' : 'already'),
            (active=='1' ? 'on' : 'off'));
    } else if (on == '1') {
        active = 1;
        PushData('D 1');
        this.Log('Position control activated');
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
    if (avrs[0]) avrs[0].SendCmd("halt\n");
    PushData('D 0');
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
            avrsi.interfaces[0].release(true, function(error) { });
            avrsi.close();
        }
        catch (err) {
        }
    }
}

//-----------------------------------------------------------------------------
// end
