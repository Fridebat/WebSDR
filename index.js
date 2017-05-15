// file name = index.js
// (c) 2017 JH1OOD/Mike

const bufa = new Buffer('fefe80e017'            , 'hex'); // preamble
const bufz = new Buffer('fd'                    , 'hex'); // postamble
const buf1 = new Buffer('fefe80e0174351fd'      , 'hex'); // CQ
const buf2 = new Buffer('fefe80e01751525a3ffd'  , 'hex'); // QRZ?
const buf3 = new Buffer('fefe80e003fd'          , 'hex'); // read freq
var   buf4 = new Buffer('fefe80e0050000000000fd', 'hex'); // send freq
 
var SerialPort = require('serialport');
var serial     = new SerialPort(
    '/dev/ttyUSB0',
    {baudrate : 19200, parser : SerialPort.parsers.byteDelimiter([ 0xfd ])});
 
var http  = require('http');
var fs    = require('fs');
var index = fs.readFileSync(__dirname + '/index.html');
var app   = http.createServer(function(req, res) {
                res.writeHead(200, {'Content-Type' : 'text/html'});
                res.end(index);
              })
              .listen(3000);
var io    = require('socket.io').listen(app);

var freqHz  = 0;          // VFO frequency
var ndata   = 512 + 2048; // waterfall_data[512] + sound_data[2048]
var pos     = 0;
var myarray = new Array();
var mywater = new Array();
var mysound = new Array();
 
// -- water fall --
 
process.stdin.on('readable', function() { // from spinor_audio
  var buf = process.stdin.read();
  if (buf !== null) {
      for(var i=0;i<buf.length;i++) {
        myarray[pos++] = buf[i];
        if (pos == ndata) {
          for(var j=0;j<512;j++) {     // waterfall_data
            mywater[j] = myarray[j];
          }
          for(var j=0;j<2048;j++) {    // sound_data
            var tmp = myarray[j+512];
            if (tmp>128) tmp -=256;
            mysound   [j] = tmp / 128.0; // signed char to -1.0~+1.0
          }
          io.emit('waterfall', mywater);
          io.emit('sound'    , mysound);
          pos = 0;
        }
      }
  }
});
 
// -- serial for IC-7410 --
 
serial.on('open',
          function() { console.error('serial port /dev/ttyUSB0 is opened.'); });
 
serial.on('data', function(data) {
  if (!(data[0] == 0xfe & data[1] == 0xfe)) {
    console.error('-- received serial data format error');
    }
  if (data[2] == 0xe0 & data[3] == 0x80 & data[4] == 0x03) {
    var f10   = data[5] >> 4 & 0x0f;
    var f1    = data[5] & 0x0f;
    var f1k   = data[6] >> 4 & 0x0f;
    var f100  = data[6] & 0x0f;
    var f100k = data[7] >> 4 & 0x0f;
    var f10k  = data[7] & 0x0f;
    var f10m  = data[8] >> 4 & 0x0f;
    var f1m   = data[8] & 0x0f;
    var freq  = f10m.toString() + f1m.toString() + "," + f100k.toString() +
                f10k.toString() + f1k.toString() + "." + f100.toString()  +
                f10.toString()  + f1.toString()  + "kHz";
    freqHz = f10m * 10000000 + f1m * 1000000 + f100k * 100000 + f10k * 10000 +
             f1k * 1000 + f100 * 100 + f10 * 10 + f1;
    io.emit('freqmsg', 'VFO: ' + freq);
  }
});
 
serial.on('error', function(err) { console.error('Error: ', err.message); })
 
// -- set frequency --
 
    function setfreq(f) {
      var f10m  = Math.floor(f / 10000000); f -= f10m  * 10000000;
      var f1m   = Math.floor(f / 1000000);  f -= f1m   * 1000000;
      var f100k = Math.floor(f / 100000);   f -= f100k * 100000;
      var f10k  = Math.floor(f / 10000);    f -= f10k  * 10000;
      var f1k   = Math.floor(f / 1000);     f -= f1k   * 1000;
      var f100  = Math.floor(f / 100);      f -= f100  * 100;
      var f10   = Math.floor(f / 10);       f -= f10   * 10;
      var f1    = Math.floor(f / 1);
 
      var data = new Array(
          [ 0xfe, 0xfe, 0x80, 0xe0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd ]);
      buf4[5] = f10   << 4 | f1;
      buf4[6] = f1k   << 4 | f100;
      buf4[7] = f100k << 4 | f10k;
      buf4[8] = f10m  << 4 | f1m;
      buf4[9] = 0;
      serial.write(buf4);
    }
 
// -- WebSocket --
 
io.on('connection', function(socket) {
 
  socket.on('message1', function() { serial.write(buf1); });
  socket.on('message2', function() { serial.write(buf2); });
 
  socket.on('message3', function() {
    var newfreq = freqHz - 10000;
    setfreq(newfreq);
  });

  socket.on('message4', function() {
    var newfreq = freqHz - 5000;
    setfreq(newfreq);
  });
 
  socket.on('message5', function() {
    var newfreq = freqHz - 2000;
    setfreq(newfreq);
  });
 
  socket.on('message6', function() {
    var newfreq = freqHz + 2000;
    setfreq(newfreq);
  });
 
  socket.on('message7', function() {
    var newfreq = freqHz + 5000;
    setfreq(newfreq);
  });
 
  socket.on('message8', function() {
    var newfreq = freqHz + 10000;
    setfreq(newfreq);
  });
 
// the following console.log() are to send signal to sprig_audio

  socket.on('message66', function(cwpitch) {
    console.log('b', cwpitch);
  });

  socket.on('message67', function(cwpitch) {
    console.log('c', cwpitch);
  });

  socket.on('message77', function(mx) {
    console.log('a', mx);
  });

  socket.on('your message', function(msg) {
    serial.write(bufa);
    serial.write(msg );
    serial.write(bufz);
    io.emit('your message', msg);
  });
 
});
 
// -- request freq --
 
function sendTime() {
  serial.write(buf3);
  // console.error('Freq asked..');
}

setInterval(sendTime, 100);

// -- EOF --
