const { parentPort } = require('worker_threads');
const ROSLIB = require('roslib');
const WebSocket = require('ws');

let ros;
let reconnectInterval = 5000; // ระยะเวลาในการลองเชื่อมต่อใหม่ (ms)
let rosbridgeURL = '';
let reconnectTimer = null;
let rosAutoConnected = false;

let cameraSocket = null;

const BUFFER_SIZE_VOLTAGE = 50;  // ค่าเฉลี่ยจาก 10 วินาที
let voltageBuffer = [];


parentPort.on('message', (message) => {
  try {
    switch (message.type) {
      case 'connectROS':
        connectROSBridge(message.url);
        break;
      case 'sendDrive':
        sendDrive(message.command);
        break;
      case 'sendCmd':
        sendCommand(message.command);
        break;
      case 'sendServo':
        sendServo(message.command);
        break;
      case 'sendRelay':
        sendRelayViaCommand(message.relayId, message.command);
        break;
      case 'listMaps':
        callListMapsService();
        break;
      case 'loadMap':
        callLoadMapService(message.mapName);
        break;
      case 'requestMapFileAsBase64':
        requestMapFileAsBase64(message.mapName);
        break;
      case 'saveMap':
        callSaveMapService(message.mapName);
        break;
      case 'sendPatrolPath':
        sendPatrolPathToROS(message.path);
        break;
      case 'sendStopPatrol':
        sendStopPatrolCommand();
        break;
      case 'sendSingleGoal':
        sendSingleGoalToMoveBase(message.point);
        break;
      case 'resumePatrol':
        resumePatrolFrom(message.path, message.index);
        break;
      case 'startSLAM':
        callStartSLAMService();
        break;
      case 'stopSLAM':
        callStopSLAMService();
        break;
      default:
        console.warn(`Server worker ❌ Unknown command: ${message.type}`);
    }
  } catch (err) {
    console.error(`Server: ❗ Worker Error while processing message [${message.type}]:`, err.message);
    // คุณสามารถเลือกส่งกลับ frontend ได้ด้วย เช่น:
    // parentPort.postMessage({ type: 'error', data: `Error: ${err.message}` });
  }
});


// 🌐 ฟังก์ชันเชื่อมต่อ ROSBridge
function connectROSBridge(url) {
  rosAutoConnected = true;
  
  
  if (ros && ros.isConnected && rosbridgeURL === url) {
    console.log('Server : ✅ Already connected to ROSBridge at ', url);
    //parentPort.postMessage({ type: 'log', data: 'Connected to ROSBridge' });
    return;
  }

  if (ros) {
    console.log('Server : 🔄 Closing previous ROSBridge connection before reconnecting...');
    //parentPort.postMessage({ type: 'log', data: 'Server : Closing previous ROSBridge connection before reconnecting...' });
    ros.close();
  }


  rosbridgeURL = url;
  ros = new ROSLIB.Ros({ 
    url: url,
    encoding: 'ascii'
  });

  ros.on('connection', () => {
    console.log('Serverosbridger : ✅ Connected to ROSBridge at', url);
    parentPort.postMessage({ type: 'connection', data: 'connected' });
    //subscribe function
    subscribeSensorData();
    subscribePatrolStatus();
    subscribeMapData();
    subscribeRobotPose();
    subscribePlannedPath();
    if (reconnectTimer) {20
      clearInterval(reconnectTimer);
      reconnectTimer = null;
      console.log('Server : 🛑 Reconnect attempts stopped after successful connection at', url);
    }
  });

  ros.on('error', (error) => {
    console.log('Server : ❌ Error connecting to ROSBridge:');
    //console.log('Server : ❌ Error connecting to ROSBridge:', error);
    parentPort.postMessage({ type: 'connection', data: 'error' });
    startReconnect();
  });

  ros.on('close', () => {
    console.log('Server : 🔌❌ Connection to ROSBridge closed url : ',url);
    parentPort.postMessage({ type: 'connection', data: 'disconnected' });
    startReconnect();
  });
}

function startReconnect() {
  if (!reconnectTimer) {
    console.log(`Server : 🔄 Attempting to reconnect to ROSBridge every ${reconnectInterval / 1000} seconds...`);
    reconnectTimer = setInterval(() => {
      if (!ros.isConnected) {
        console.log('Server : 🔗 Reconnecting to ROSBridge at', rosbridgeURL);
        connectROSBridge(rosbridgeURL); // ✅ ใช้ IP ล่าสุดที่รับเข้ามา
      } else {
        clearInterval(reconnectTimer);
        reconnectTimer = null;
      }
    }, reconnectInterval);
  }
}

function sendRelayViaCommand(relayId, command) {
  const relayCommandMap = {
    relay1: {
      on:  0x08000001,
      off: 0x08000000
    },
    relay2: {
      on:  0x08000003,
      off: 0x08000002
    }
  };

  const cmdValue = relayCommandMap[relayId]?.[command];
  if (cmdValue === undefined) {
    console.error(`Server : ❌ Unknown relay command: ${relayId}, ${command}`);
    return;
  }

  console.log(`Server : 📤 Relay ${relayId} ${command.toUpperCase()} → HEX: ${cmdValue.toString(16)} → DEC: ${cmdValue}`);
  sendCommand(cmdValue);
}


// 📤 ส่งคำสั่ง UInt32 Command ไปยัง ROSBridge สำหรับคำสั่งต่างๆ
function sendCommand(command) {
  if (!ros || !ros.isConnected) {
    console.error('Server : ❌ Cannot send command: ROSBridge is not connected.');
    return;
  }
  const uint32Value = command >>> 0;
  console.log(`Server : 📤 Sending UInt32 Command: ${uint32Value}`);

  const cmdEditTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/rb/cm/ed',
    messageType: 'std_msgs/UInt32',
  });

  const message = new ROSLIB.Message({
    data: uint32Value,
  });

  cmdEditTopic.publish(message);
}

// 📤 ส่งคำสั่ง UInt16 Command ไปยัง ROSBridge สำหรับการเคลื่อนไหวของล้อ
function sendDrive(command) {
  const uint16Value = command & 0xFFFF; // ให้แน่ใจว่าอยู่ในช่วง 16-bit
  console.log(`Server : Sending uint16 Command: ${uint16Value}`);
  const cmdVelTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/rb/cm/dr',
    messageType: 'std_msgs/UInt16'
  });

  const message = new ROSLIB.Message({
    data: uint16Value
  });

  cmdVelTopic.publish(message);
}

// 📤 ส่งคำสั่ง UInt8 Command ไปยัง ROSBridge สำหรับการเคลื่อนไหวของ servo
function sendServo(command) {
  const uint8Value = command & 0xFF; // ให้แน่ใจว่าอยู่ในช่วง 8-bit
  console.log(`Server : Sending uint8 Command: ${uint8Value}`);
  const cmdVelTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/rb/cm/sv',
    messageType: 'std_msgs/UInt8'
  });

  const message = new ROSLIB.Message({
    data: uint8Value
  });

  cmdVelTopic.publish(message);
}
// 🗺️ Subscribe ข้อมูลแผนที่จาก ROS
function subscribeMapData() {
  const mapTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/map',
    messageType: 'nav_msgs/OccupancyGrid',
  });

  mapTopic.subscribe((msg) => {
    parentPort.postMessage({
      type: 'live-map',
      data: msg
    });
  });
}
function subscribeRobotPose() {
  const poseTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/amcl_pose',
    messageType: 'geometry_msgs/PoseWithCovarianceStamped'
  });

  poseTopic.subscribe((msg) => {
    const pos = msg.pose.pose.position;
    const ori = msg.pose.pose.orientation;

    parentPort.postMessage({
      type: 'robot-pose',
      data: { position: pos, orientation: ori }
    });
  });
}
function subscribePlannedPath() {
  const planTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/move_base/NavfnROS/plan', 
    messageType: 'nav_msgs/Path'
  });

  planTopic.subscribe((message) => {
    // แปลงข้อมูล poses ให้เป็น array ของ {x, y}
    const pathPoints = message.poses.map(p => ({
      x: p.pose.position.x,
      y: p.pose.position.y
    }));

    parentPort.postMessage({
      type: 'planned-path',
      data: pathPoints
    });
  });
}

// Power
function subscribeSensorData() {
  const sensorTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/sensor/data',
    messageType: 'std_msgs/UInt32',
  });

  sensorTopic.subscribe((message) => {
    const raw = message.data >>> 0;
    const current_mA = (raw >> 16) & 0xFFFF;
    const voltage_cV = raw & 0xFFFF;

    const voltage_V = voltage_cV / 100.0;
    const current_A = current_mA / 1000.0;

    // เติมค่าใหม่เข้า buffer
    voltageBuffer.push(voltage_V);
    if (voltageBuffer.length > BUFFER_SIZE_VOLTAGE) {
        voltageBuffer.shift(); // ลบค่าที่เก่าที่สุดออก
    }

    // คำนวณค่าเฉลี่ย
    let sum = 0;
    for (let i = 0; i < voltageBuffer.length; i++) {
        sum += voltageBuffer[i];
    }
    let avgVoltage = 0;
    if (voltageBuffer.length > 0) {
        avgVoltage = sum / voltageBuffer.length;
    } else {
        avgVoltage = 0; // ค่า default 
    }



    // คำนวณ % SOC แบบ linear (12.0V = 0%, 14.6V = 100%)
    let soc = 0;
    const maxVoltage = 13.50;
    const minVoltage = 12.60;
    if (avgVoltage >= maxVoltage) soc = 100;
    else if (avgVoltage <= minVoltage) soc = 0;
    else soc = ((avgVoltage - minVoltage) / (maxVoltage - minVoltage)) * 100;

    parentPort.postMessage({
      type: 'power',
      data: {
        voltage: voltage_V.toFixed(2),
        current: current_A.toFixed(2),
        percent: soc.toFixed(0)
      }
    });
  });
}

// service call สำหรับ list_maps
function callListMapsService() {
  if (!ros || !ros.isConnected) {
    parentPort.postMessage({ type: 'map-list', data: [], error: 'ROSBridge not connected' });
    return;
  }

  const service = new ROSLIB.Service({
    ros: ros,
    name: '/map_manager/list_maps',
    serviceType: 'ptR1_navigation/ListMaps'
  });

  const request = new ROSLIB.ServiceRequest({});
  service.callService(request, (result) => {
    parentPort.postMessage({ type: 'map-list', data: result.names });
  }, (err) => {
    console.error('❌ list_maps service failed:', err);
    parentPort.postMessage({ type: 'map-list', data: [], error: err.toString() });
  });
}
// service call สำหรับ load_map ให้เป็น active map
function callLoadMapService(mapName) {
  if (!ros || !ros.isConnected) {
    parentPort.postMessage({ type: 'map-load', data: { success: false, message: 'ROSBridge not connected' } });
    return;
  }

  const service = new ROSLIB.Service({
    ros: ros,
    name: '/map_manager/load_map',
    serviceType: 'ptR1_navigation/LoadMap'
  });

  const request = new ROSLIB.ServiceRequest({ name: mapName });

  service.callService(request, (result) => {
  parentPort.postMessage({
    type: 'map-load',
    data: {
      ...result,
      name: mapName  // ✅ ใส่ชื่อ map ที่โหลดอยู่
    }
  });
}, (err) => {
  console.error('❌ load_map service failed:', err);
  parentPort.postMessage({
    type: 'map-load',
    data: {
      success: false,
      message: err.toString(),
      name: mapName  // แม้ error ก็ยังส่งชื่อกลับ
    }
  });
});

}
// service call สำหรับ map_file จากชื่อ
function requestMapFileAsBase64(mapName) {
  const service = new ROSLIB.Service({
    ros,
    name: '/map_manager/get_map_file',
    serviceType: 'ptR1_navigation/GetMapFile'
  });

  const request = new ROSLIB.ServiceRequest({ name: mapName });

  service.callService(request, (res) => {
    if (res.success) {
      parentPort.postMessage({
        type: 'map-data',
        data: {
          name: mapName,
          base64: res.image_data_base64,
          yaml: res.yaml_data 
        }
      });
    } else {
      console.warn(`❌ Map fetch failed: ${res.message}`);
    }
  });
}

function callSaveMapService(mapName) {
  if (!ros || !ros.isConnected) {
    parentPort.postMessage({
      type: 'map-save',
      data: { success: false, message: 'ROSBridge not connected', name: mapName }
    });
    return;
  }

  const service = new ROSLIB.Service({
    ros: ros,
    name: '/map_manager/save_map',
    serviceType: 'ptR1_navigation/SaveMap'
  });

  const request = new ROSLIB.ServiceRequest({ name: mapName });

  service.callService(request, (result) => {
    parentPort.postMessage({
      type: 'map-save',
      data: {
        ...result,
        name: mapName
      }
    });
  }, (err) => {
    console.error('❌ save_map service failed:', err);
    parentPort.postMessage({
      type: 'map-save',
      data: {
        success: false,
        message: err.toString(),
        name: mapName
      }
    });
  });
}
function callStartSLAMService() {
  if (!ros || !ros.isConnected) {
    parentPort.postMessage({
      type: 'slam-result',
      data: { success: false, message: 'ROSBridge not connected' }
    });
    return;
  }

  const service = new ROSLIB.Service({
    ros: ros,
    name: '/start_slam',
    serviceType: 'std_srvs/Trigger'  // หรือเปลี่ยนตามของคุณ
  });

  const request = new ROSLIB.ServiceRequest({});

  service.callService(request, (res) => {
    parentPort.postMessage({
      type: 'slam-result',
      data: { success: res.success, message: res.message }
    });
  }, (err) => {
    parentPort.postMessage({
      type: 'slam-result',
      data: { success: false, message: err.toString() }
    });
  });
}
function callStopSLAMService() {
  const service = new ROSLIB.Service({
    ros: ros,
    name: '/map_manager/stop_slam',
    serviceType: 'ptR1_navigation/StopSLAM'
  });

  const req = new ROSLIB.ServiceRequest({});
  service.callService(req, (res) => {
    parentPort.postMessage({
      type: 'slam-stop-result',
      data: { success: res.success, message: res.message }
    });
  }, (err) => {
    parentPort.postMessage({
      type: 'slam-stop-result',
      data: { success: false, message: err.toString() }
    });
  });
}
function sendPatrolPathToROS(pathArray) {
  if (!ros || !ros.isConnected) {
    console.warn("ROS ยังไม่เชื่อมต่อ");
    return;
  }

  const topic = new ROSLIB.Topic({
    ros: ros,
    name: '/patrol_path',
    messageType: 'geometry_msgs/PoseArray'
  });

  const poses = pathArray.map(pt => ({
    position: { x: pt.x, y: pt.y, z: 0 },
    orientation: { x: 0, y: 0, z: 0, w: 1 } // หรือคำนวณ quaternion
  }));

  const msg = new ROSLIB.Message({
    header: { frame_id: 'map' },
    poses: poses
  });

  topic.publish(msg);
  console.log('📤 ส่ง PoseArray ไปยัง /patrol_path');
}
// ฟัง topic feedback เช่น /patrol_status (Bool หรือ String ก็ได้)
function subscribePatrolStatus() {
  const patrolStatusTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/patrol_status',
    messageType: 'std_msgs/Bool'
  });

  patrolStatusTopic.subscribe((msg) => {
    const isMoving = msg.data;
    parentPort.postMessage({ type: 'patrol-status', data: isMoving });
  });
}
function sendStopPatrolCommand() {
  if (!ros || !ros.isConnected) {
  console.warn('Server : ❌ Cannot stop patrol – ROSBridge not connected.');
  return;
  }
  const stopTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/stop_patrol',
    messageType: 'std_msgs/Bool'
  });

  const msg = new ROSLIB.Message({ data: true });
  stopTopic.publish(msg);
  console.log('🛑 หยุดการลาดตระเวน');
}
function sendSingleGoalToMoveBase(pt) {
  if (!ros || !ros.isConnected) return;

  const goalTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/rb/navigation/goal',
    messageType: 'geometry_msgs/PoseStamped',
  });

  const msg = new ROSLIB.Message({
    header: { frame_id: 'map', stamp: new Date() },
    pose: {
      position: { x: pt.x, y: pt.y, z: 0 },
      orientation: { x: 0, y: 0, z: 0, w: 1 }
    }
  });

  console.log(`📍 ส่ง goal เดี่ยวไปยัง (${pt.x.toFixed(2)}, ${pt.y.toFixed(2)})`);
  goalTopic.publish(msg);
}
function resumePatrolFrom(pathArray, startIndex) {
  if (!ros || !ros.isConnected) return;

  if (!Array.isArray(pathArray) || startIndex >= pathArray.length) {
    console.warn("❌ Invalid resume path or index");
    return;
  }

  const subPath = pathArray.slice(startIndex);

  const topic = new ROSLIB.Topic({
    ros: ros,
    name: '/patrol_path',
    messageType: 'geometry_msgs/PoseArray'
  });

  const poses = subPath.map(pt => ({
    position: { x: pt.x, y: pt.y, z: 0 },
    orientation: { x: 0, y: 0, z: 0, w: 1 }
  }));

  const msg = new ROSLIB.Message({
    header: { frame_id: 'map' },
    poses: poses
  });

  topic.publish(msg);
  console.log(`▶️ Resumed patrol from index ${startIndex} (${subPath.length} points)`);
}
setTimeout((url) => {
  if (!rosAutoConnected) {
    console.log('Server : ⏳ No IP received in 3s, connecting to localhost fallback...');
  }
}, 3000);

parentPort.postMessage({ type: 'log', data: 'Worker Initialized' });

module.exports = {
  connectROSBridge,
  sendDrive,
  sendCommand,
  sendServo
};
