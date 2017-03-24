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
const kHistoryLen       = 600;

const kBanner = '---- CUTE Cryostat Position Control ' + kServerVersion + ' ----';

var evkSN = [
    'ffffffff3850313339302020ff0d0b',   // EVK 0
    'ffffffff3850313339302020ff1011'    // EVK 1
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

var evks = [];          // EVK devices
var foundEVKs = 0;      // number of recognized EVKs
var conn = [];          // http client connections
var active = 0;         // flag set if position control is active

var flashPin = ['pa07','pa08','pa21','pa08'];   // sequence to flash LED's
var flashNum = flashPin.length - 1;
var flashTime = 0;

var vals = [0,0,0];     // most recent measured values
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

        // This is the most important callback for us, we'll handle
        // all messages from users here.
        connection.on('message', function(message) {
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
                var i = message.utf8Data.indexOf(':');
                var cmd;
                if (i > 0) {
                    cmd = message.utf8Data.substr(0,i).trim();
                    str = message.utf8Data.substr(i+1).trim();
                } else {
                    cmd = message.utf8Data.trim();
                    str = '';
                }
                switch (cmd.toLowerCase()) {
                    case 'help':
                        this.Respond('Available commands: help, name, log, who, active');
                        break;
                    case 'name':
                        if (str.length) {
                            this.Log('Name = "'+str+'"');
                            this.cuteName = str;
                        } else {
                            this.Respond('Your name is "'+this.cuteName+'"');
                        }
                        break;
                    case 'log':
                        this.Log(str);
                        break;
                    case 'who':
                        this.Respond('Current users: ' +
                            conn.map(function(e){
                                return e.cuteName + (e==this?' (you)':'')
                            },this).join(', '));
                        break;
                    case 'active':
                        this.Activate(str);
                        break;
                    default:
                        this.Respond('Unknown command: ' + cmd);
                        break;
                }
            } else if (message.type === 'binary') {
                // handle binary data here (message.binaryData)
                this.Respond("Received binary data length=" + message.binaryData.length);
            } else {
                this.Respond('Received unknown message type=' + message.type);
            }
        });

        // add new connection to our list
        conn[conn.length] = connection;
        connection.cuteName = GetAddr(request);

        // define a few convenience functions
        connection.Activate = function (str) { Activate.call(this,str); };
        connection.SendData = function (str) { this.send(str, function ack(error) { }); };
        connection.Respond = function (str) { this.SendData('C '+EscapeHTML(str)+'<br/>'); };
        connection.Log = function () { Log('['+this.cuteName+'] '+Array.from(arguments).join(' ')); };

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
        connection.Respond('(' + foundEVKs + ' EVKs connected)');

        connection.Log('Connected');
        // send message indicating whether or not we are active
        connection.send('D ' + active, function ack(error) { });

        // send measurement history (packet "B")
        for (var i=history.length-1; i>=0; --i) {
            connection.send('B '+((historyTime - i) % kHistoryLen)+' '+
                         history[i][0].toFixed(4)+' '+
                         history[i][1].toFixed(4)+' '+
                         history[i][2].toFixed(4), function ack(error) {
                // If error is not defined, the send has been completed,
                // otherwise the error object will indicate what failed.
            });
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
        OpenEVK(device);
    }
});

// handle USB devices that have disconnected
usb.on('detach', ForgetEVK);

Log();
Log(kBanner);

FindEVKs();     // find all connected EVK boards

// poll the hardware periodically
intrvl = setInterval(function() {
    for (var i=0; i<evks.length; ++i) {
        if (evks[i] == null) continue;
        if (i < 2) {
            evks[i].SendToEVK("d.adc2;d.adc3\n");
        } else {
            // re-send "ser" command in case it was missed
            evks[i].SendToEVK("a.ser;b.ver;c.wdt 1\n");
        }
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
// Send data to all http clients
function BroadcastData(str)
{
    for (var i=0; i<conn.length; ++i) {
        conn[i].SendData(str);
    }
}

//-----------------------------------------------------------------------------
// Activate/deactivate position control
function Activate(arg)
{
    var on = { 'off':0, '0':0, 'on':1, '1':1 }[arg.toLowerCase()];
    if (arg == '' || on == active) {
        this.Respond('Active control is ' + (on==null ? 'currently' : 'already') +
            ' ' + (active=='1' ? 'on' : 'off'));
    } else if (on=='1' || on=='0') {
        active = on;
        BroadcastData('D ' + on);
        this.Log('Position control', active==1 ? 'activated' : 'deactivated');
    } else {
        this.Respond('Invalid argument for "active" command');
    }
}

//-----------------------------------------------------------------------------
// Open communication with our EVK device
function OpenEVK(device)
{
    try {
        device.open();
        device.interfaces[0].claim();
        // find first unused input >= 2
        for (var i=2; ; ++i) {
            if (i < evks.length && evks[i]) continue;
            evks[i] = device;
            device.evkNum = i;
            var endIn  = device.interfaces[0].endpoints[0];
            var endOut = device.interfaces[0].endpoints[1];
            endIn.evkNum  = i;
            endOut.evkNum = i;
            endIn.transferType  = usb.LIBUSB_TRANSFER_TYPE_BULK;
            endOut.transferType = usb.LIBUSB_TRANSFER_TYPE_BULK;
            endIn.timeout  = 1000;
            endOut.timeout = 1000;
            endIn.startPoll(4, 256);
            endIn.on('data', HandleData);
            endIn.on('error', HandleError);
            device.SendToEVK = function SendToEVK(cmd) {
                try {
                    this.interfaces[0].endpoints[1].transfer(cmd, function(error) {
                        if (error) {
                            Log('EVK', this.evkNum, 'send error');
                            ForgetEVK(this);
                        }
                    });
                }
                catch (err) {
                    Log('EVK', this.evkNum, 'send exception');
                    ForgetEVK(this);
                }
            };
            device.SendToEVK("a.ser;b.ver;c.wdt 1\n");
            break;
        }
    }
    catch (err) {
        Log('Error opening EVK device');
    }
}

//-----------------------------------------------------------------------------
// Forget about this EVK
function ForgetEVK(deviceOrEndpoint)
{
    var evkNum = deviceOrEndpoint.evkNum;
    if (evkNum == null) {
        Log('Unknown EVK detached!');
    } else if (evks[evkNum]) {
        Log('EVK', evkNum, 'detached!');
        if (evkNum < 2) --foundEVKs;
        evks[evkNum] = null;
    }
}

//-----------------------------------------------------------------------------
// Handle USB errors
function HandleError(err)
{
    // if (err) Log(err);
}

//-----------------------------------------------------------------------------
// Receive data from our EVK devices
function HandleData(data)
{
    var evkNum = this.evkNum;
    if (evkNum == null) {
        Log('Data from unknown device!');
        return;
    }
    var str = data.toString();
    // wait for null at end of string
    var j = str.indexOf('\0');
    if (j > -1) str = str.substr(0, j); // remove null
    // process each response separately
    var lines = str.split("\n");
    for (j=0; j<lines.length; ++j) {
        var str = lines[j];
        if (!str.length) continue;
        if (str.length < 4 || (str.substr(1,1) != '.') || str.substr(2,2) != 'OK') {
            // (we sometimes get truncated responses from the first couple of commands
            // sent before the EVK has fully initialized, but these don't cause problems,
            // so just ignore these errors for now)
            // Log('EVK', evkNum, 'Command error:', str);
        } else {
            var msg = str.length > 5 ? str.substr(5) : '';
            var id = str.substr(0,1);
            evkNum = HandleResponse(evkNum, id, msg);
        }
    }
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
            if (history.length > kHistoryLen) history.pop();
        }
        entry = history[0];
    }
    for (var j=0; j<vals.length; ++j) {
        entry[j+i] = Number(vals[j]);
    }
    return t;
}

//-----------------------------------------------------------------------------
// Handle response from an EVK command
// Returns: new EVK number (may change in response to a "ser" command)
function HandleResponse(evkNum, responseID, msg)
{
    switch (responseID) {
        case 'a':   // a = get serial number
            var evk;
            for (var i=0; i<evkSN.length; ++i) {
                if (msg == evkSN[i]) {
                    evk = 'EVK ' + i;
                    if (evkNum < 2 || !evks[evkNum]) {
                        if (evks[i] != evks[evkNum]) {
                            Log(evk, 'INTERNAL ERROR');
                        } else {
                            // could get here if we got a "ser" response
                            // from an already identified EVK
                            return evkNum; // (nothing to do)
                        }
                    } else {
                        if (evks[i]) {
                            if (evks[i] != evks[evkNum]) {
                                Log(evk, 'ERROR! Already exists!');
                            }
                        } else {
                            ++foundEVKs;
                        }
                        // change the number of this EVK
                        evks[i] = evks[evkNum];
                        evks[i].evkNum = i;
                        evks[i].interfaces[0].endpoints[0].evkNum = i;
                        evks[i].interfaces[0].endpoints[1].evkNum = i;
                        evks[evkNum] = null;
                        evkNum = i;
                    }
                    break;
                }
            }
            if (!evk) {
                evk = 'Unknown EVK ' + evkNum;
                evks[evkNum].SendToEVK('z.wdt 0\n');  // disable watchdog on unknown EVK
            }
            Log(evk, 'attached (s/n', msg + ')');
            break;

        case 'b':   // b = log this response
            Log('EVK', evkNum, msg);
            break;

        case 'c':   // c = ignore this response
            break;

        case 'd':   // d = handle ADC read responses
            var j = msg.indexOf('VAL=');
            if (j >= 0) {
                // ie. "adc2 512"
                switch (msg.substr(3,1)) {
                    case '2':
                        vals[evkNum*2] = msg.substr(j+4) / 900;
                        break;
                    case '3':
                        vals[evkNum*2+1] = msg.substr(j+4) / 545;
                        // save in history and send data back to http clients
                        // after reading last adc from EVK 1
                        if (evkNum) {
                            var t = AddToHistory(0, vals);
                            BroadcastData('A '+ (t % kHistoryLen)+' '+
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
                                    if (evks[i]) evks[i].SendToEVK(cmd);
                                }
                                flashNum = flashNew;
                                flashTime = t;
                            }
                        }
                        break;
                }
            }
            break;

        case 'z':   // z = disable watchdog timer
            // forget about the unknown EVK
            evks[evkNum] = null;
            break;

        default:
            Log('EVK', evkNum, 'Unknown response:', msg);
            break;
    }
    return evkNum;
}

//-----------------------------------------------------------------------------
// Enumerate USB devices to locate our EVK boards
function FindEVKs()
{
    var devs = usb.getDeviceList();
    for (var i=0; i<devs.length; ++i) {
        if (devs[i].deviceDescriptor.idVendor  == kAtmel &&
            devs[i].deviceDescriptor.idProduct == kEVK1101)
        {
            OpenEVK(devs[i]);
        }
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
    BroadcastData('C ' +  EscapeHTML(msg) + '<br/>');
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

    for (var i=0; i<evks.length; ++i) {
        if (!evks[i]) continue;
        var evki = evks[i];
        evks[i] = null;
        if (i < 2) --foundEVKs;
        // release any still-open interfaces
        try {
            evksi.interfaces[0].release(true, function (error) { });
            evksi.close();
        }
        catch (err) {
        }
    }
}

//-----------------------------------------------------------------------------
// end
