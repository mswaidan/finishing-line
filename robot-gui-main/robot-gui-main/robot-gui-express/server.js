const express = require('express')
const { createServer } = require('node:http')
const { join } = require('node:path')
const { Server } = require('socket.io')
const struct = require('python-struct')

const app = express()
const server = createServer(app)
const io = new Server(server)

var net = require('net')
var ccClient = new net.Socket()
const urIp = '192.168.1.32' // Replace with your robot's IP address
const PORT = 30001          // Use port 30003 for real-time data stream

app.use(express.static(join(__dirname, 'public')))
app.use('/public', express.static(join(__dirname, 'public')))
app.use(express.static(join(__dirname, 'public', 'css')))

app.get('/', (req, res) => {
  res.sendFile(join(__dirname, 'index.html'))
})
server.listen(3000, () => {
  console.log('server running at http://localhost:3000')
})

const sendTcpMessage = (msg) => {
  ccClient.connect(8888, '192.168.1.17', function () {
    console.log('Connected')
    ccClient.write(msg)
    ccClient.on('data', function (data) {
      console.log('Received: ' + data)
      ccClient.destroy()
    })
  })
}

// set up a socket connection to the UR primary interface
function primaryConnect(){ 
    // Create a TCP client
    const primaryClient = new net.Socket()
    
    // Function to connect to the robot and request the mode
    primaryClient.connect(PORT, urIp, () => {
      console.log(`Connected to robot at ${urIp}:${PORT}`)
      primaryClient.setKeepAlive(true, 10000); // Enable keep-alive with a 10-second interval
    })
    
    // Handle incoming data (robot's response)
    primaryClient.on('data', (data) => {
      const pkgType = struct.unpack('!b', data.subarray(4, 5))[0]
      if (pkgType == 16) handleStatePackage(data)
    })
    
    // Handle connection closure
    primaryClient.on('close', () => {
      console.log('Connection closed, attempting to reconnect...');
      reconnect();
    })
    
    // Handle errors
    primaryClient.on('error', (err) => {
      console.error('Connection error:', err.message);
      reconnect();
    })
}

function reconnect() {
  setTimeout(primaryConnect, 5000); // Attempt to reconnect after 5 seconds
}

primaryConnect()

io.on('connection', (socket) => {
  if (unpacked) {
    socket.emit('robot state', unpacked.slice(3))
  }
  console.log('a user connected')
  socket.on('job message', (msg) => {
    console.log('job message: ' + msg)
    sendTcpMessage(msg)
    socket.emit('active job', msg)
  })
  socket.on('run message', (msg) => {
    console.log('run message: ' + msg)
    sendTcpMessage(msg)
    socket.emit('conveyor state', msg)
  })
})

let lastStateMsg = ''
let unpacked

const handleStatePackage = (data) => {
  const msgLength = struct.unpack('!i', data.subarray(0, 4))[0]
  const msgType = struct.unpack('!B', data.subarray(4, 5))[0]
  let i = 0

  while (i + 5 < msgLength) {
    const subMsgLength = struct.unpack('!i', data.subarray(5 + i, 9 + i))[0]
    const subMsgType = struct.unpack('!B', data.subarray(9 + i, 10 + i))[0]
    const subMsg = data.subarray(5 + i, 5 + i + subMsgLength)

    switch (subMsgType) {
      case 0: // state message
        unpacked = struct.unpack('!iBQ7?2B3dB', subMsg)
        if (unpacked.slice(3) != lastStateMsg) {
          // broadcast new state
          io.emit('robot state', unpacked.slice(3))

          console.log("isRealRobotConnected: " + unpacked[3])
          console.log("isRealRobotEnabled: " + unpacked[4])
          console.log("isRobotPowerOn: " + unpacked[5])
          console.log("isEmergencyStopped: " + unpacked[6])
          console.log("isProtectiveStopped: " + unpacked[7])
          console.log("isProgramRunning: " + unpacked[8])
          console.log("isProgramPaused: " + unpacked[9])
          console.log("\n")
          lastStateMsg = unpacked.slice(3).toString()
        }
    }

    i += subMsgLength
  }
}