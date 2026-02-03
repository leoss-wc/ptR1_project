const { parentPort } = require('worker_threads');
const ROSLIB = require('roslib');
const { CMD } = require('../main/constants.js');

let ros;
let reconnectInterval = 5000; // ระยะเวลาในการลองเชื่อมต่อใหม่ (ms)
let rosbridgeURL = '';
let reconnectTimer = null;
let rosAutoConnected = false;

let slamPoseSubscriber = null;
let amclPoseSubscriber = null;
let isSlamPoseInitialized = false; 

const BUFFER_SIZE_VOLTAGE = 50;  // ค่าเฉลี่ยจาก 10 วินาที
let voltageBuffer = [];
let tfClient = null;


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
      case 'sendSingleGoal':
        sendSingleGoalToMoveBase(message.data); 
        break;
      case 'startSLAM':
        callStartSLAMService();
        break;
      case 'stopSLAM':
        callStopSLAMService();
        break;
      case 'setInitialPose':
        publishInitialPose(message.pose);
        break;
      case 'startStream':
        callStartStreamService();
        break;
      case 'stopStream':
        callStopStreamService();
        break;
      case 'switchPoseSubscriber':
        console.log(`Server: Switching pose subscriber to mode: ${message.mode}`);
        
        if (amclPoseSubscriber) {
          amclPoseSubscriber.unsubscribe();
          amclPoseSubscriber = null;
        }
        if (slamPoseSubscriber) {
          slamPoseSubscriber.unsubscribe();
          slamPoseSubscriber = null;
        }
        if (message.mode === 'amcl') {
          subscribeAmclPose();
        } else if (message.mode === 'slam') {
          subscribeRobotPoseSlam();
        }
        break;

      case 'deleteMap':
        callDeleteMapService(message.mapName);
        break;
      case 'resetSLAM':
        callResetSLAMService();
        break;
      case 'startPatrol':
        callStartPatrolService(message.goals, message.loop);
        break;
      case 'pausePatrol':
        callPausePatrolService();
        break;
      case 'resumePatrol':
        callResumePatrolService();
        break;
      case 'stopPatrol':
        callStopPatrolService();
        break;
      case 'getParam': 
        getRosParam(message.name); break;
      case 'setParam': 
        setRosParam(message.name, message.value); break;
      default:
        console.warn(`Server worker  Unknown command: ${message.type}`);
    }
  } catch (err) {
    console.error(`Server: Worker Error while processing message [${message.type}]:`, err.message);
  }
});


// ฟังก์ชันเชื่อมต่อ ROSBridge
function connectROSBridge(url) {
  console.log('Server : Connecting to ROSBridge at ', url);
  rosAutoConnected = true;
  
  
  if (ros && ros.isConnected && rosbridgeURL === url) {
    console.log('Server : Already connected to ROSBridge at ', url);
    //parentPort.postMessage({ type: 'log', data: 'Connected to ROSBridge' });
    return;
  }

  if (ros) {
    console.log('Server : Closing previous ROSBridge connection before reconnecting...');
    //parentPort.postMessage({ type: 'log', data: 'Server : Closing previous ROSBridge connection before reconnecting...' });
    ros.close();
  }


  rosbridgeURL = url;
  ros = new ROSLIB.Ros({ 
    url: url,
    encoding: 'ascii'
  });

  ros.on('connection', () => {
    console.log('Serverosbridger : Connected to ROSBridge at', url);
    parentPort.postMessage({ type: 'connection', data: 'connected' });
    //subscribe function
    subscribeSensorData();
    subscribeMapData();
    subscribePlannedPath();
    subscribeMoveBaseResult();
    subscribeLaserScanData();
    subscribeTF();
    if (reconnectTimer) {20
      clearInterval(reconnectTimer);
      reconnectTimer = null;
      console.log('Server : Reconnect attempts stopped after successful connection at', url);
    }
  });

  ros.on('error', (error) => {
    console.log('Server : Error connecting to ROSBridge:');
    parentPort.postMessage({ type: 'connection', data: 'error' });
    startReconnect();
  });

  ros.on('close', () => {
    console.log('Server :  Connection to ROSBridge closed url : ',url);
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
      on:  CMD.RELAY1_ON,
      off: CMD.RELAY1_OFF
    },
    relay2: {
      on:  CMD.RELAY2_ON,
      off: CMD.RELAY2_OFF
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

// ส่งคำสั่ง UInt32 Command ไปยัง ROSBridge สำหรับคำสั่งต่างๆ
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

//ส่งคำสั่ง UInt16 Command ไปยัง ROSBridge สำหรับการเคลื่อนไหวของล้อ
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

//ส่งคำสั่ง UInt8 Command ไปยัง ROSBridge สำหรับการเคลื่อนไหวของ servo
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
//Subscribe ข้อมูลแผนที่จาก ROS
function subscribeMapData() {
  const mapTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/map',
    messageType: 'nav_msgs/OccupancyGrid',
    throttle_rate: 1000
  });

  mapTopic.subscribe((msg) => {
    if (!isSlamPoseInitialized) {
      console.log('Server: First map message received, initializing SLAM pose subscription.');
      isSlamPoseInitialized = true; // ตั้งค่า flag เป็น true เพื่อไม่ให้ทำงานซ้ำ
      
      // หน่วงเวลาเล็กน้อยเพื่อให้แน่ใจว่าระบบพร้อมก่อน subscribe
      setTimeout(() => {
        subscribeRobotPoseSlam();
      }, 200);
    }
    parentPort.postMessage({
      type: 'live-map',
      data: msg
    });
  });
}
function subscribeSlamMapData() {
  const slamMapTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/rb/slam/map', 
    messageType: 'nav_msgs/OccupancyGrid',
  });

  console.log(`[Server] Subscribing to SLAM map topic: ${slamMapTopic.name}`);

  slamMapTopic.subscribe((msg) => {
    // ส่งข้อมูลแผนที่จาก SLAM ผ่าน Message Type ใหม่
    parentPort.postMessage({
      type: 'slam-map-update', // ✅ ใช้ Type ใหม่!
      data: msg
    });
  });
}
function subscribeRobotPoseSlam() {
  if (!ros || !ros.isConnected) return;
  console.log('Server: Subscribing to SLAM pose topic /robot_pose_sample...');

  // ตรวจสอบและยกเลิก subscriber เก่า ถ้ามี
  if (slamPoseSubscriber) {
    slamPoseSubscriber.unsubscribe();
  }

  // สร้าง Topic object ใหม่และ "เก็บค่า" ไว้ในตัวแปร slamPoseSubscriber
  slamPoseSubscriber = new ROSLIB.Topic({
    ros: ros,
    name: '/robot_pose_sample',
    messageType: 'geometry_msgs/PoseStamped'
  });

  slamPoseSubscriber.subscribe((msg) => {
    const pos = msg.pose.position;
    const ori = msg.pose.orientation;
    
    parentPort.postMessage({
      type: 'robot-pose-slam',
      data: { position: pos, orientation: ori }
    });
  });
}
function subscribeAmclPose() {
  if (!ros || !ros.isConnected) return;
  console.log('Server: Subscribing to AMCL pose topic /amcl_pose...');

  // ถ้ามี subscriber เก่าอยู่ ให้ยกเลิกก่อน
  if (amclPoseSubscriber) {
    amclPoseSubscriber.unsubscribe();
  }

  amclPoseSubscriber = new ROSLIB.Topic({
    ros: ros,
    name: '/amcl_pose',
    messageType: 'geometry_msgs/PoseWithCovarianceStamped'
  });

  amclPoseSubscriber.subscribe((msg) => {
    const pos = msg.pose.pose.position;
    const ori = msg.pose.pose.orientation;
    parentPort.postMessage({
      type: 'robot-pose-amcl',
      data: { position: pos, orientation: ori }
    });
  });
}
function subscribeLaserScanData() {
  if (!ros || !ros.isConnected) return;

  const scanTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/scan', // ชื่อ Topic ของ Laser Scan โดยทั่วไป
    messageType: 'sensor_msgs/LaserScan',
    throttle_rate : 2000 // ลดความถี่การส่งข้อมูลเหลือ 1 ครั้งต่อวินาที
    
  });

  console.log('[Server] Subscribing to LaserScan topic: /scan');

  scanTopic.subscribe((message) => {
    // ส่งข้อมูลที่จำเป็นกลับไปเท่านั้น เพื่อลดขนาดข้อมูล
    parentPort.postMessage({
      type: 'laser-scan-update',
      data: {
        angle_min: message.angle_min,
        angle_increment: message.angle_increment,
        ranges: message.ranges
      }
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

function subscribeMoveBaseResult() {
  if (!ros || !ros.isConnected) return;

  const resultTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/move_base/result',
    messageType: 'move_base_msgs/MoveBaseActionResult'
  });

  resultTopic.subscribe((msg) => {
    if (!msg.status) return;

    let result = { status: 'UNKNOWN', text: msg.status.text || '' };

    switch (msg.status.status) {
      case 0: // PENDING
        console.log('Server: Goal is PENDING.');
        // สถานะนี้ไม่ปรากฏใน /result topic แต่ใส่ไว้เพื่อความสมบูรณ์
        result.status = 'PENDING';
        break;
      case 1: // ACTIVE
        console.log('Server: Goal is ACTIVE.');
        // สถานะนี้ไม่ปรากฏใน /result topic
        result.status = 'ACTIVE';
        break;
      case 2: // PREEMPTED
        //ถูกยกเลิก (โดย Goal ใหม่): Goal ปัจจุบันถูกยกเลิก เพราะมี Goal ใหม่ถูกส่งเข้ามาแทนที่
        console.warn('Server: Goal PREEMPTED (cancelled by a new goal).');
        result.status = 'PREEMPTED';
        break;
      case 3: // SUCCEEDED
        // สำเร็จ: หุ่นยนต์ไปถึงเป้าหมายที่กำหนดไว้
        console.log('Server: Goal SUCCEEDED.');
        result.status = 'SUCCEEDED';
        break;
      case 4: // ABORTED
        // ล้มเหลว: หุ่นยนต์ไม่สามารถไปถึงเป้าหมายได้ (เช่น มีสิ่งกีดขวาง)
        console.error('Server: Goal ABORTED (failed to reach).');
        result.status = 'ABORTED';
        break;
      case 5: // REJECTED
        // ถูกปฏิเสธ: เป้าหมายถูกปฏิเสธโดย action server (เช่น เป้าหมายอยู่นอกขอบเขตที่กำหนด)
        console.error('Server: Goal REJECTED (invalid goal).');
        result.status = 'REJECTED';
        break;
      case 6: // PREEMPTING
        // กำลังถูกยกเลิก: อยู่ในระหว่างกระบวนการยกเลิกเป้าหมาย
        console.log('Server: Goal is PREEMPTING (cancellation in progress).');
        result.status = 'PREEMPTING';
        break;
      case 7: // RECALLING
        //  กำลังร้องขอยกเลิก: มีการส่งคำขอยกเลิก Goal ไปแล้ว แต่ Server ยังไม่ตอบรับ
        console.log('Server: Goal is RECALLING (cancellation requested).');
        result.status = 'RECALLING';
        break;
      case 8: // RECALLED
        //ยกเลิกสำเร็จ: Server ยืนยันว่า Goal นี้ถูกยกเลิกเรียบร้อยแล้ว
        console.warn('Server: Goal RECALLED (cancelled successfully).');
        result.status = 'RECALLED';
        break;
      case 9: // LOST
        //การเชื่อมต่อขาดหาย: การสื่อสารกับ Action Server ที่ทำงานนี้อยู่ขาดหายไป
        console.error('Server: Goal LOST (action server disappeared).');
        result.status = 'LOST';
        break;
      default:
        console.warn(`Server: Goal finished with unhandled status: ${msg.status.status}`);
        break;
    }

    // ส่งผลลัพธ์ทั้งหมดกลับไปที่ Main Process ผ่าน Event เดียว
    if (result.status !== 'UNKNOWN' && result.status !== 'ACTIVE' && result.status !== 'PENDING') {
      parentPort.postMessage({ type: 'goal-result', data: result });
    }
  });
}

// Power
function subscribeSensorData() {
  const sensorTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/sensor/data',
    messageType: 'std_msgs/UInt32',
    throttle_rate: 500
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
    name: '/map_manager/start_slam',
    serviceType: 'ptR1_navigation/StartSLAM' 
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
    name: '/map_manager/stop_processes',
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

function sendSingleGoalToMoveBase(data) {
  if (!ros || !ros.isConnected) return;

  const goalTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/move_base_simple/goal',
    messageType: 'geometry_msgs/PoseStamped',
  });

  const msg = new ROSLIB.Message({
    header: { frame_id: 'map' },
    pose: {
      position: data.pose.position,
      orientation: data.pose.orientation
    }
  });

  console.log(`📍 ส่ง goal (พร้อมทิศทาง) ไปยัง (${data.pose.position.x.toFixed(2)}, ${data.pose.position.y.toFixed(2)})`);
  goalTopic.publish(msg);
}

function publishInitialPose(pose) {
  if (!ros || !ros.isConnected) {
    console.error('Server : ❌ Cannot send initial pose: ROSBridge is not connected.');
    return;
  }

  const initialPoseTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/initialpose',
    messageType: 'geometry_msgs/PoseWithCovarianceStamped'
  });

  const message = new ROSLIB.Message({
    header: {
      frame_id: 'map'
    },
    pose: {
      pose: {
        position: {
          x: pose.position.x,
          y: pose.position.y,
          z: 0
        },
        orientation: pose.orientation
      },
      // Covariance บอกถึงความไม่แน่นอน (ค่ามาตรฐานที่ใช้กันทั่วไป)
      covariance: [
        0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
      ]
    }
  });

  console.log('Server : 📤 Publishing to /initialpose:', message);
  initialPoseTopic.publish(message);
}

function callStartStreamService() {
  if (!ros || !ros.isConnected) {
    console.log('Server : Start Stream Service Failed: ROS is not connected.');
    // ส่งข้อความกลับไปว่าล้มเหลวเนื่องจากไม่ได้เชื่อมต่อ
    parentPort.postMessage({
      type: 'stream-status',
      data: {
        success: false,
        message: 'ROS is not connected.'
      }
    });
    return; // ออกจากฟังก์ชัน
  }
  const service = new ROSLIB.Service({
    ros: ros,
    name: '/stream_manager/start',
    serviceType: 'std_srvs/Trigger'
  });
  service.callService(new ROSLIB.ServiceRequest({}), (result) => {
    console.log('Server : Start Stream Service Result:', result);
    parentPort.postMessage({ type: 'stream-status', data: result });
  });
}

function callStopStreamService() {
  if (!ros || !ros.isConnected) {
    console.log('Server : Start Stream Service Failed: ROS is not connected.');
    // ส่งข้อความกลับไปว่าล้มเหลวเนื่องจากไม่ได้เชื่อมต่อ
    parentPort.postMessage({
      type: 'stream-status',
      data: {
        success: false,
        message: 'ROS is not connected.'
      }
    });
    return;
  }
  const service = new ROSLIB.Service({
    ros: ros,
    name: '/stream_manager/stop',
    serviceType: 'std_srvs/Trigger'
  });
  service.callService(new ROSLIB.ServiceRequest({}), (result) => {
    console.log('Server : Stop Stream Service Result:', result);
    parentPort.postMessage({ type: 'stream-status', data: result });
  });
}

function callDeleteMapService(mapName) {
  if (!ros || !ros.isConnected) {
    // ส่งผลลัพธ์กลับไปที่ UI ผ่าน main process
    parentPort.postMessage({ type: 'map-delete-result', data: { success: false, message: 'ROSBridge not connected' } });
    return;
  }
  const service = new ROSLIB.Service({
    ros: ros,
    name: '/map_manager/delete_map',
    serviceType: 'ptR1_navigation/DeleteMap' // <--- ใช้ Service Type ที่ถูกต้อง
  });
  const request = new ROSLIB.ServiceRequest({ name: mapName });
  service.callService(request, (result) => {
    parentPort.postMessage({ type: 'map-delete-result', data: result });
  }, (err) => {
    parentPort.postMessage({ type: 'map-delete-result', data: { success: false, message: err.toString() } });
  });
}

function callResetSLAMService() {
  if (!ros || !ros.isConnected) {
    parentPort.postMessage({ type: 'slam-reset-result', data: { success: false, message: 'ROSBridge not connected' } });
    return;
  }
  const service = new ROSLIB.Service({
    ros: ros,
    name: '/map_manager/reset_slam',
    serviceType: 'ptR1_navigation/ResetSLAM' // <--- ใช้ Service Type ที่ถูกต้อง
  });
  const request = new ROSLIB.ServiceRequest({});
  service.callService(request, (result) => {
    parentPort.postMessage({ type: 'slam-reset-result', data: result });
  }, (err) => {
    parentPort.postMessage({ type: 'slam-reset-result', data: { success: false, message: err.toString() } });
  });
}

function callStartPatrolService(goals, loop) {
  if (!ros || !ros.isConnected) {
    parentPort.postMessage({ type: 'patrol-start-result', data: { success: false, message: 'ROS is not connected.' } });
    return;
  }
  const service = new ROSLIB.Service({
    ros,
    name: '/map_manager/start_patrol',
    serviceType: 'ptR1_navigation/StartPatrol'
  });
  const request = new ROSLIB.ServiceRequest({ goals, loop });
  service.callService(request, (result) => {
    parentPort.postMessage({ type: 'patrol-start-result', data: result });
  }, (err) => {
    parentPort.postMessage({ type: 'patrol-start-result', data: { success: false, message: err.toString() } });
  });
}

function callPausePatrolService() {
  if (!ros || !ros.isConnected) return;
  const service = new ROSLIB.Service({ ros, name: '/map_manager/pause_patrol', serviceType: 'ptR1_navigation/PausePatrol' });
  service.callService(new ROSLIB.ServiceRequest({}), (result) => {
    parentPort.postMessage({ type: 'patrol-pause-result', data: result });
  });
}

function callResumePatrolService() {
  if (!ros || !ros.isConnected) return;
  const service = new ROSLIB.Service({ ros, name: '/map_manager/resume_patrol', serviceType: 'ptR1_navigation/ResumePatrol' });
  service.callService(new ROSLIB.ServiceRequest({}), (result) => {
    parentPort.postMessage({ type: 'patrol-resume-result', data: result });
  });
}

function callStopPatrolService() {
  if (!ros || !ros.isConnected) return;
  const service = new ROSLIB.Service({ ros, name: '/map_manager/stop_patrol', serviceType: 'ptR1_navigation/StopPatrol' });
  service.callService(new ROSLIB.ServiceRequest({}), (result) => {
    parentPort.postMessage({ type: 'patrol-stop-result', data: result });
  });
}

function subscribeTF() {
  if (!ros || !ros.isConnected) return;

  console.log('Server : 📡 Initializing TF Client...');

  // สร้าง TF Client โดยระบุว่าเรายึด 'map' เป็นเฟรมหลัก
  tfClient = new ROSLIB.TFClient({
    ros: ros,
    fixedFrame: 'map',
    angularThres: 0.01, // อัปเดตเมื่อมุมเปลี่ยน 0.01 rad
    transThres: 0.01,   // อัปเดตเมื่อระยะเปลี่ยน 0.01 เมตร
    rate: 10.0          // ความถี่สูงสุด 10Hz
  });

  // Subscribe หาตำแหน่งของ 'base_link' (ตัวหุ่น) เทียบกับ 'map'
  tfClient.subscribe('base_link', (tf) => {
    // tf จะมีข้อมูล translation (x,y,z) และ rotation (x,y,z,w)
    
    // ส่งข้อมูลกลับไปที่ Main Process (เพื่อส่งต่อให้ Frontend วาด)
    parentPort.postMessage({
      type: 'tf-update',
      data: {
        translation: tf.translation,
        rotation: tf.rotation
      }
    });
  });
}

parentPort.postMessage({ type: 'log', data: 'Worker Initialized' });
