//-----------------------------------------------------------------------------
//
// File:        cute_server.js
//
// Description: CUTE cryostat position control hardware interface
//
// Revisions:   2017-03-16 - P. Harvey created
//
// Syntax:      node cute_server.js
//
// Notes:       Requires additional node libraries to run.  Install them
//              by typing: "npm install usb"
//
const kAtmel   = 0x03eb;
const kEVK1101 = 0x2300;
const kHistoryLen = 600;

var evkSN = [
    'ffffffff3850313339302020ff0d0b',   // EVK 0
    'ffffffff3850313339302020ff1011'    // EVK 1
];

var authorized = {
    '*' : 1, // (uncomment this to allow commands from any IP)
    'localhost' : 1,
    '130.15.24.88' : 1,
};

var ver = '0.01';

var usb = require('usb');
var fs = require('fs');
var WebSocketServer = require('websocket').server;
var http = require('http');
var vals = [0,0,0];

var evks = [];          // EVK devices
var inpt = [];          // EVK in endpoints
var outpt = [];         // EVK out endpoints
var foundEVKs = 0;      // number of recognized EVKs
var conn = [];          // http client connections
var active = 0;

// each element of the history array is an array with the following elements:
// 0-2) damper heights
// 3-5) damper forces
// 6-8) damper nominal forces
// 9) atm pressure 
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
            if (message.type === 'utf8') {
                var addr = GetAddr(this);
                if (!authorized[addr] && !authorized['*']) {
                    Log("["+addr+"] Client has no command authority");
                    return;
                }
                var n = message.utf8Data.indexOf(':');
                var cmd, str;
                if (n > 0) {
                    cmd = message.utf8Data.substr(0,n);
                    str = message.utf8Data.substr(n+1);
                } else {
                    cmd = message.utf8Data;
                    str = '';
                }
                switch (cmd) {
                    case 'Log':
                        break;
                    case 'On':
                        Activate(1,addr);
                        return;
                    case 'Off':
                        Activate(0,addr);
                        return;
                    default:
                        str = 'Unknown command: ' + message.utf8Data;
                        break;
                }
            } else if (message.type === 'binary') {
                // handle binary data here (message.binaryData)
                str = "Received binary data length=" + message.binaryData.length;
            } else {
                str = 'Received unknown message type=' + message.type;;
            }
            Log(str);
        });

        conn[conn.length] = connection;
    
        connection.on('close', function(reason) {
            // close user connection
            Log('[' + GetAddr(this) + '] Closed connection');
            for (var i=0; i<conn.length; ++i) {
                if (conn[i] == this) {
                    conn.splice(i,1); // remove from list
                    break;
                }
            }
        });

        connection.send('C ---- CUTE Cryostat Position Control v' + ver + ' ----<br/>(' + foundEVKs + ' EVKs connected)<br/>',
            function ack(error) { });

        Log('[' + GetAddr(request) + '] Connected');
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
Log('---- CUTE Cryostat Position Control v' + ver + ' ----');

FindEVKs();     // find all connected EVK boards

// poll the hardware periodically
intrvl = setInterval(function() {
    for (var i=0; i<inpt.length; ++i) {
        if (inpt[i] == null) continue;
        if (i < 2) {
            Send(i, "d.adc2;d.adc3\n");
        } else {
            // re-send "ser" command in case it was missed
            Send(i, "a.ser;b.ver;c.pa31 -\n");
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
function SendAll(str)
{
    for (var i=0; i<conn.length; ++i) {
        conn[i].send(str, function ack(error) { });
    }
}

//-----------------------------------------------------------------------------
// Activate/deactivate position control
function Activate(on, addr)
{
    if (active != on) {
        active = on;
        SendAll('D ' + on);
        Log('[' + addr + '] Position control ' + (active==1 ? 'activated' : 'deactivated'));
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
            if (i < inpt.length && inpt[i]) continue;
            inpt[i] = device.interfaces[0].endpoints[0];
            outpt[i] = device.interfaces[0].endpoints[1];
            inpt[i].transferType = usb.LIBUSB_TRANSFER_TYPE_BULK;
            outpt[i].transferType = usb.LIBUSB_TRANSFER_TYPE_BULK;
            inpt[i].timeout = 1000;
            outpt[i].timeout = 1000;
            evks[i] = device;
            inpt[i].startPoll(4, 1024);
            inpt[i].on('data', HandleData);
            inpt[i].on('error', HandleError);
            Send(i, "a.ser;b.ver;c.pa31 -\n");
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
    for (var i=0; i<evks.length; ++i) {
        if (deviceOrEndpoint == evks[i] || deviceOrEndpoint == outpt[i]) {
            Log('EVK', i, 'detached!');
            evks[i] = inpt[i] = outpt[i] = null;
            if (i < 2) --foundEVKs;
            return;
        }
    }
}

//-----------------------------------------------------------------------------
// Send command to an EVK
function Send(evkNum, cmd)
{
    outpt[evkNum].transfer(cmd, function(error) {
        if (error) {
            ForgetEVK(this);
        }
    });
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
    var evkNum = -1;
    for (var i=0; i<evks.length; ++i) {
        if (inpt[i] == this) {
            evkNum = i;
            break;
        }
    }
    if (evkNum < 0) {
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
            Log('EVK', evkNum, 'Command error:', str);
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
                        evks[i] = evks[evkNum];
                        inpt[i] = inpt[evkNum];
                        outpt[i] = outpt[evkNum];
                        evks[evkNum] = inpt[evkNum] = outpt[evkNum] = null;
                        evkNum = i;
                    }
                    break;
                }
            }
            if (!evk) {
                evk = 'Unknown EVK';
                Send(evkNum, 'z.wdt 0\n');  // disable watchdog on unknown EVK
            }
            Log(evk, 'attached (s/n', msg + ')');
            break;

        case 'b':   // b = ver
            Log('EVK', evkNum, msg);
            break;

        case 'c':   // c = nop
            break;

        case 'd':   // d = adc2 command
            var j = msg.indexOf('VAL=');
            if (j >= 0) {
                // ie. "adc2 512"
                switch (msg.substr(3,1)) {
                    case '2':
                        vals[evkNum*2] = msg.substr(j+4) / 900;
                        break;
                    case '3':
                        vals[evkNum*2+1] = msg.substr(j+4) / 545;
                        // send data back to http clients after reading last adc from EVK 0
                        var t = AddToHistory(0, vals);
                        if (evkNum) {
                            SendAll('A '+ (t % kHistoryLen)+' '+
                                           vals[0].toFixed(4)+' '+
                                           vals[1].toFixed(4)+' '+
                                           vals[2].toFixed(4)+' '+
                                           vals[3].toFixed(4));
                        }
                        break;
                }
            }
            break;

        case 'z':   // z = disable watchdog timer
            // forget about the unknown EVK
            evks[evkNum] = inpt[evkNum] = outpt[evkNum] = null;
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
            var used;
            for (var j=0; j<evks.length; ++j) {
                if (devs[i] == evks[j]) {
                    used = 1;
                    break;
                }
            }
            if (!used) OpenEVK(devs[i]);
        }
    }
}

//-----------------------------------------------------------------------------
// Log message to console and file
function Log()
{
    var msg;
    if (arguments.length) {
        var d = new Date();
        msg = d.getFullYear()    + '-' + Pad2(d.getMonth()+1) + '-' + Pad2(d.getDate())    + ' ' +
              Pad2(d.getHours()) + ':' + Pad2(d.getMinutes()) + ':' + Pad2(d.getSeconds()) + ' ' +
              Array.from(arguments).join(' ');
    } else {
        msg = '';
    }
    console.log(msg);
    fs.appendFile('cute_server.log', msg + '\n', function(error) {
        if (error) console.log(error, 'writing log file');
    });
    // send back to clients
    SendAll('C ' +  msg + '<br/>', function ack(error) { });
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
        evks[i] = inpt[i] = outpt[i] = null;
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
