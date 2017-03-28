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

const kBanner = '---- CUTE Cryostat Position Control ' + kServerVersion + ' ----';

var avrSN = [
    'ffffffff3850313339302020ff0d0b',   // AVR0
    'ffffffff3850313339302020ff1011'    // AVR1
];

var authorized = {
    '*' : 1, // (uncomment this to allow commands from any IP)
    'localhost' : 1,
    '130.15.24.88' : 1
};

var usb = require('usb');
var fs = require('fs');
var WebSocketServer = require('websocket').server;
var http = require('http');

var avrs = [];          // AVR devices
var foundAVRs = 0;      // number of recognized AVRs
var conn = [];          // http client connections
var active = 0;         // flag set if position control is active

var flashPin = ['pa07','pa08','pa21','pa08'];   // sequence to flash LED's
var flashNum = flashPin.length - 1;
var flashTime = 0;

var vals = [0,0,0];     // most recent measured values
var motorSpd = [0,0,0]; // motor speeds
var lastSpd = '0 0 0';  // last motor speeds sent to clients (as string)
var motorPos = [0,0,0]; // motor positions
var history = [];       // history of measured values
var historyTime = -1;   // time of most recent history entry

var intrvl;             // interval timer for polling hardware

//-----------------------------------------------------------------------------
// Main script

var server = http.createServer(function(request, response) {
    // process HTTP request. Since we're writing just WebSockets server
    // we don't have to implement anything.
});
server.on('error', function(error) {
    Log('Server error!');
});

server.listen(8080, function() { });

// create the server
wsServer = new WebSocketServer({
    httpServer: server
});

// WebSocket server
wsServer.on('request', function(request) {
    try {
        var connection = request.accept("cute", request.origin);

        // add new connection to our list
        conn[conn.length] = connection;
        connection.cuteName = GetAddr(request);

        // define a few member functions
        connection.Activate = Activate;
        connection.SendData = function (str) {
            this.send(str, function ack(error) { });
        };
        connection.Respond = function () {
            this.SendData('C <span class=res>'+EscapeHTML(Array.from(arguments).join(' '))+'</span><br/>');
        };
        connection.Log = function () {
            Log('['+this.cuteName+'] '+Array.from(arguments).join(' '));
        };
        connection.HandleServerCommand = HandleServerCommand;

        // handle all messages from users here
        connection.on('message', function(message) {
            this.HandleServerCommand(message);
        });

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
});

// handle CTRL-C and SIGINT signal
process.on('SIGINT', function() {
    console.log('');    // (for linefeed after CTRL-C)
    Log('Exiting on SIGINT');
    Cleanup();
    // exit after giving time to log last message
    setTimeout(function() { process.exit(); }, 10);
});

// handle new USB devices being connected
usb.on('attach', function(device) {
    if (device.deviceDescriptor.idVendor  == kAtmel &&
        device.deviceDescriptor.idProduct == kEVK1101)
    {
        OpenAVR(device);
    }
});

// handle USB devices that have disconnected
usb.on('detach', ForgetAVR);

Log();
Log(kBanner);

FindAVRs();     // find all connected AVR boards

// poll the hardware periodically
intrvl = setInterval(function() {
    for (var i=0; i<avrs.length; ++i) {
        if (avrs[i] == null) continue;
        var cmd;
        switch (i) {
            case 0: // AVR0
                cmd = "d.adc2;adc3;f.m0;m1;m2\n";
                break;
            case 1: // AVR1
                cmd = "d.adc2;adc3\n";
                break;
            default:
                // re-send "ser" command in case it was missed
                cmd = "a.ser;b.ver\n";
                break;
        }
        avrs[i].SendCmd(cmd);
    }
}, 100);

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
            cmd = message.utf8Data.substr(0,i).trim();
            str = message.utf8Data.substr(i+1).trim();
        } else {
            cmd = message.utf8Data.trim();
            str = '';
        }
        // process the command
        switch (cmd.toLowerCase()) {
            case 'help':
                this.Respond('Available commands: help, name, who, log, active, list, avr#');
                break;
            case 'name':
                if (str.length) {
                    this.Log('/'+cmd, str);
                    this.cuteName = str;
                } else {
                    this.Respond('Your name is "'+this.cuteName+'"');
                }
                break;
            case 'who':
                this.Respond('Current users:',
                    conn.map(function(e){
                        return e.cuteName + (e==this?' (you)':'')
                    },this).join(', '));
                break;
            case 'list': {
                var num = 0;
                for (var i=0; i<avrs.length; ++i) if (avrs[i]) ++num;
                this.Respond(num, 'AVRs connected:');
                for (var i=0; i<avrs.length; ++i) {
                    if (avrs[i]) this.Respond('AVR'+i, avrs[i].avrSN);
                }
            }   break;
            case 'active':
                this.Activate(str);
                break;
            case 'log':
                this.Log(str);
                break;
            default:
                // check for AVR commands
                if (cmd.length==4 && cmd.substr(0,3).toLowerCase() == 'avr') {
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
// Activate/deactivate position control
function Activate(arg)
{
    var on = { 'off':0, '0':0, 'on':1, '1':1 }[arg.toLowerCase()];
    if (arg == '' || on == active) {
        this.Respond('Active control is', (on==null ? 'currently' : 'already'),
            (active=='1' ? 'on' : 'off'));
    } else if (on=='1' || on=='0') {
        active = on;
        PushData('D ' + on);
        this.Log('Position control', active==1 ? 'activated' : 'deactivated');
    } else {
        this.Respond('Invalid argument for "active" command');
    }
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
    return str.replace(/[&<>]/g, function (s) { return entityMap[s]; });
}

//-----------------------------------------------------------------------------
// Log message to console and file
function Log()
{
    var msg;
    var d = new Date();
    var date = d.getFullYear() + '-' + Pad2(d.getMonth()+1) + '-' + Pad2(d.getDate());
    if (arguments.length) {
        msg = date + ' ' + Pad2(d.getHours()) + ':' + Pad2(d.getMinutes()) + ':' +
                Pad2(d.getSeconds()) + ' ' + Array.from(arguments).join(' ');
    } else {
        msg = '';
    }
    console.log(msg);
    fs.appendFile('cute_server_'+date+'.log', msg + '\n', function(error) {
        if (error) console.log(error, 'writing log file');
    });
    // send back to clients
    PushData('C ' +  EscapeHTML(msg) + '<br/>');
}

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
                        avrNum = i;
                    }
                    break;
                }
            }
            if (avr) {
                avrs[avrNum].SendCmd('c.wdt 1\n');  // enable watchdog timer
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
            if (j >= 0) {
                // ie. "adc2 512"
                switch (msg.substr(3,1)) {
                    case '2':
                        vals[avrNum*2] = msg.substr(j+4) / 900;
                        break;
                    case '3':
                        vals[avrNum*2+1] = msg.substr(j+4) / 545;
                        // save in history and send data back to http clients
                        // after reading last adc from AVR1
                        if (avrNum) {
                            var t = AddToHistory(0, vals);
                            PushData('A '+ (t % kPosHisLen)+' '+
                                vals[0].toFixed(4)+' '+
                                vals[1].toFixed(4)+' '+
                                vals[2].toFixed(4)+' '+
                                vals[3].toFixed(4));
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
                        break;
                }
            }
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
                if (n==2) {
                    var newSpd = motorSpd.join(' ');
                    if (lastSpd != newSpd) {
                        PushData('E ' + newSpd);
                        lastSpd = newSpd;
                    }
                }
            }
        }   break;

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
    for (var j=0; j<vals.length; ++j) {
        entry[j+i] = vals[j];
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
            avrsi.interfaces[0].release(true, function (error) { });
            avrsi.close();
        }
        catch (err) {
        }
    }
}

//-----------------------------------------------------------------------------
// end
