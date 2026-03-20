const { parentPort } = require('worker_threads');
const ROSLIB = require('roslib');
const { CMD } = require('../main/constants.js');
const { handleCaptureMessage, initCaptureSubscriber } = require('./captureServer');

let ros;
let reconnectInterval = 5000;
let rosbridgeURL = '';
let reconnectTimer = null;

let slamPoseSubscriber = null;
let amclPoseSubscriber = null;
let isSlamPoseInitialized = false;

let tfClient = null;

// ── Cached publish topics  ──────────────────────────────────
let _cmdTopic        = null;
let _twistTopic      = null;
let _servoTiltTopic  = null;
let _servoPanTopic   = null;
let _goalTopic       = null;
let _initialPoseTopic = null;

// ── Pose dedup state ──────────────────────────────────────────────────────────
const POSE_MIN_DIST  = 0.01;   // เมตร  (~1 cm)
const POSE_MIN_ANGLE = 0.01;   // เรเดียน
let lastSlamPos   = null;
let lastSlamYaw   = null;
let lastAmclPos   = null;
let lastAmclYaw   = null;

// ── Map dedup state ───────────────────────────────────────────────────────────
let lastMapSeq = null;

// ── Laser downsample ──────────────────────────────────────────────────────────
const LASER_STEP = 3; // เก็บทุก 3 ค่า → ลด payload ~67%

// ─────────────────────────────────────────────────────────────────────────────
// Helper: ดึง yaw จาก quaternion
function quatToYaw(o) {
  return Math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z));
}

// Helper: กรอง message ที่เก่าเกินไป (ป้องกัน message สะสมตอนเน็ตสะดุดแล้วโยนมาทีเดียว)
const MAX_MSG_AGE_MS = 2000; // ทิ้ง message ที่เก่ากว่า 2 วินาที
function isFresh(stamp) {
  if (!stamp) return true; // ถ้าไม่มี stamp ให้ผ่านเสมอ
  const msgTimeMs = stamp.secs * 1000 + stamp.nsecs / 1e6;
  return (Date.now() - msgTimeMs) < MAX_MSG_AGE_MS;
}

// Helper: เช็คว่า pose เปลี่ยนพอจะส่งหรือเปล่า
function poseChanged(pos, yaw, lastPos, lastYaw) {
  if (!lastPos) return true;
  const dx = pos.x - lastPos.x;
  const dy = pos.y - lastPos.y;
  const dist = Math.sqrt(dx * dx + dy * dy);
  const dYaw = Math.abs(yaw - (lastYaw ?? 0));
  return dist >= POSE_MIN_DIST || dYaw >= POSE_MIN_ANGLE;
}

// ─────────────────────────────────────────────────────────────────────────────
parentPort.on('message', (message) => {
  try {
    switch (message.type) {
      case 'connectROS':
        connectROSBridge(message.url);
        break;
      case 'sendRelay':
        sendRelayViaCommand(message.command);
        break;
      case 'listMaps':
        callListMapsService();
        break;
      case 'selectNavMap':
        callSelectNavMapService(message.mapName);
        break;
      case 'requestMapFileAsBase64':
        requestMapFileAsBase64(message.mapName);
        break;
      case 'saveMap':
        callSaveMapService(message.mapName);
        break;
      case 'saveEditedMap':
        callSaveEditedMapService(message.data.name, message.data.base64, message.data.yamlContent);
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
        if (amclPoseSubscriber) { amclPoseSubscriber.unsubscribe(); amclPoseSubscriber = null; }
        if (slamPoseSubscriber) { slamPoseSubscriber.unsubscribe(); slamPoseSubscriber = null; }
        if (message.mode === 'amcl') {
          subscribeAmclPose();
        } else if (message.mode === 'slam') {
          subscribeRobotPoseSlam();
        }
        break;
      case 'deleteMap':
        callDeleteMapService(message.mapName);
        break;
      case 'startNavigation':
        callStartNavService(message.data.restorePose);
        break;
      case 'resetSLAM':
        callResetSLAMService();
        break;
      case 'stopNavigation':
        callStopNavService(message.data.savePose);
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
      case 'setHome':
        callHomeService('/nav/set_home', message.mapName, 'Set Home');
        break;
      case 'goHome':
        callHomeService('/nav/go_home', message.mapName, 'Go Home');
        break;
      case 'initHome':
        callHomeService('/nav/init_home', message.mapName, 'Init Home');
        break;
      case 'getParam':
        getRosParam(message.name);
        break;
      case 'setParam':
        setRosParam(message.name, message.value);
        break;
      case 'sendCmd':
        sendCommand(message.command);
        break;
      case 'sendTwist':
        publishTwist(message.data);
        break;
      case 'sendServoTiltInt16':
        publishServoTiltAngle(message.angle);
        break;
      case 'sendServoPanInt16':
        publishServoPanAngle(message.angle);
        break;
      case 'updateDetection':
        callUpdateDetectionService(message.data);
        break;
      case 'captureSnapshot':
      case 'captureBurstStart':
      case 'captureBurstStop':
        handleCaptureMessage(message, ros, parentPort);
        break;
      default:
        console.warn(`Server worker Unknown command: ${message.type}`);
    }
  } catch (err) {
    console.error(`Server: Worker Error while processing message [${message.type}]:`, err.message);
  }
});

// ─────────────────────────────────────────────────────────────────────────────
// เคลียร์ cached topics เมื่อ disconnect (เพื่อสร้างใหม่หลัง reconnect)
function clearCachedTopics() {
  _cmdTopic         = null;
  _twistTopic       = null;
  _servoTiltTopic   = null;
  _servoPanTopic    = null;
  _goalTopic        = null;
  _initialPoseTopic = null;

  lastSlamPos  = null; lastSlamYaw  = null;
  lastAmclPos  = null; lastAmclYaw  = null;
  lastMapSeq   = null;
}

// ─────────────────────────────────────────────────────────────────────────────
function connectROSBridge(url) {
  console.log('Server : Connecting to ROSBridge at ', url);

  if (ros && ros.isConnected && rosbridgeURL === url) {
    console.log('Server : Already connected to ROSBridge at ', url);
    return;
  }

  if (ros) {
    console.log('Server : Closing previous ROSBridge connection before reconnecting...');
    ros.close();
  }

  clearCachedTopics();
  rosbridgeURL = url;
  ros = new ROSLIB.Ros({ url, encoding: 'ascii' });

  ros.on('connection', () => {
    console.log('Server : Connected to ROSBridge at', url);
    parentPort.postMessage({ type: 'connection', data: { isConnected: true } });

    subscribeMapData();
    subscribePlannedPath();
    subscribeMoveBaseResult();
    subscribeLaserScanData();
    subscribeRobotStatus();
    subscribePatrolStatus();
    subscribeTF();
    subscribeSystemProfile();
    subscribeDetectionAlert();
    initCaptureSubscriber(ros, parentPort);

    if (reconnectTimer) {
      clearInterval(reconnectTimer);
      reconnectTimer = null;
      console.log('Server : Reconnect attempts stopped after successful connection at', url);
    }
  });

  ros.on('error', () => {
    console.log('Server : Error connecting to ROSBridge');
    parentPort.postMessage({ type: 'connection', data: { isConnected: false } });
    startReconnect();
  });

  ros.on('close', () => {
    console.log('Server : Connection to ROSBridge closed url :', url);
    clearCachedTopics();
    parentPort.postMessage({ type: 'connection', data: { isConnected: false } });
    startReconnect();
  });
}

function startReconnect() {
  if (!reconnectTimer) {
    // แจ้ง UI ว่ากำลัง connecting
    parentPort.postMessage({ 
      type: 'connection', 
      data: { isConnected: false, isConnecting: true } 
    });
    reconnectTimer = setInterval(() => {
      if (!ros.isConnected) {
        connectROSBridge(rosbridgeURL);
      } else {
        clearInterval(reconnectTimer);
        reconnectTimer = null;
      }
    }, reconnectInterval);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Relay
function sendRelayViaCommand(command) {
  const relayCommandMap = { on: CMD.RELAY_ON, off: CMD.RELAY_OFF };
  const cmdValue = relayCommandMap[command];
  if (cmdValue === undefined) {
    console.error(`Server : ❌ Unknown relay command: ${command}`);
    return;
  }
  console.log(`Server : 📤 Relay ${command.toUpperCase()} → HEX: ${cmdValue.toString(16)}`);
  sendCommand(cmdValue);
}

// ── Cached publish: /robot/cmd ────────────────────────────────────────────────
function sendCommand(command) {
  if (!ros || !ros.isConnected) {
    console.error('Server : Cannot send command: ROSBridge is not connected.');
    return;
  }
  if (!command) {
    console.error('Server : Error: Command is undefined or null');
    return;
  }
  if (!_cmdTopic) {
    _cmdTopic = new ROSLIB.Topic({ ros, name: '/robot/cmd', messageType: 'std_msgs/String' });
  }
  console.log('Server : Publishing command to /robot/cmd:', command);
  _cmdTopic.publish(new ROSLIB.Message({ data: String(command) }));
}

// ─────────────────────────────────────────────────────────────────────────────
// Map — ส่งเฉพาะเมื่อ sequence เปลี่ยน (map ใหม่จริงๆ)
function subscribeMapData() {
  const mapTopic = new ROSLIB.Topic({
    ros,
    name: '/map',
    messageType: 'nav_msgs/OccupancyGrid',
    throttle_rate: 1000
  });

  mapTopic.subscribe((msg) => {
    if (!isSlamPoseInitialized) {
      console.log('Server: First map message received, initializing SLAM pose subscription.');
      isSlamPoseInitialized = true;
      setTimeout(() => subscribeRobotPoseSlam(), 200);
    }

    // ── dedup: ข้ามถ้า map seq ยังไม่เปลี่ยน ──
    const seq = msg.header?.seq;
    if (seq !== undefined && seq === lastMapSeq) return;
    lastMapSeq = seq;

    parentPort.postMessage({ type: 'live-map', data: msg });
  });
}

// ─────────────────────────────────────────────────────────────────────────────
// SLAM Pose — ส่งเฉพาะเมื่อหุ่นขยับจริง
function subscribeRobotPoseSlam() {
  if (!ros || !ros.isConnected) return;
  console.log('Server: Subscribing to SLAM pose /robot_pose_sample...');

  if (slamPoseSubscriber) slamPoseSubscriber.unsubscribe();

  slamPoseSubscriber = new ROSLIB.Topic({
    ros,
    name: '/robot_pose_sample',
    messageType: 'geometry_msgs/PoseStamped',
    throttle_rate: 200  // รับสูงสุด 5Hz (เดิมไม่จำกัด)
  });

  slamPoseSubscriber.subscribe((msg) => {
    if (!isFresh(msg.header?.stamp)) return; // ทิ้ง message เก่าที่สะสมระหว่างเน็ตสะดุด
    const pos = msg.pose.position;
    const ori = msg.pose.orientation;
    const yaw = quatToYaw(ori);

    if (!poseChanged(pos, yaw, lastSlamPos, lastSlamYaw)) return;
    lastSlamPos = { x: pos.x, y: pos.y };
    lastSlamYaw = yaw;

    parentPort.postMessage({ type: 'robot-pose-slam', data: { position: pos, orientation: ori } });
  });
}

// ─────────────────────────────────────────────────────────────────────────────
// AMCL Pose — ส่งเฉพาะเมื่อหุ่นขยับจริง
function subscribeAmclPose() {
  if (!ros || !ros.isConnected) return;
  console.log('Server: Subscribing to AMCL pose /amcl_pose...');

  if (amclPoseSubscriber) amclPoseSubscriber.unsubscribe();

  amclPoseSubscriber = new ROSLIB.Topic({
    ros,
    name: '/amcl_pose',
    messageType: 'geometry_msgs/PoseWithCovarianceStamped',
    throttle_rate: 200  // รับสูงสุด 5Hz 
  });

  amclPoseSubscriber.subscribe((msg) => {
    if (!isFresh(msg.header?.stamp)) return; // ทิ้ง message เก่าที่สะสมระหว่างเน็ตสะดุด
    const pos = msg.pose.pose.position;
    const ori = msg.pose.pose.orientation;
    const yaw = quatToYaw(ori);

    if (!poseChanged(pos, yaw, lastAmclPos, lastAmclYaw)) return;
    lastAmclPos = { x: pos.x, y: pos.y };
    lastAmclYaw = yaw;

    parentPort.postMessage({ type: 'robot-pose-amcl', data: { position: pos, orientation: ori } });
  });
}

// ─────────────────────────────────────────────────────────────────────────────
// Laser Scan — downsample ก่อนส่ง
function subscribeLaserScanData() {
  if (!ros || !ros.isConnected) return;

  const scanTopic = new ROSLIB.Topic({
    ros,
    name: '/scan',
    messageType: 'sensor_msgs/LaserScan',
    throttle_rate: 1000
  });

  console.log('[Server] Subscribing to LaserScan: /scan');

  scanTopic.subscribe((msg) => {
    if (!isFresh(msg.header?.stamp)) return; // ทิ้ง scan เก่าที่สะสมระหว่างเน็ตสะดุด
    // เก็บทุก LASER_STEP ค่า → ลด payload ~67%
    const sampledRanges = msg.ranges.filter((_, i) => i % LASER_STEP === 0);

    parentPort.postMessage({
      type: 'laser-scan-update',
      data: {
        angle_min:       msg.angle_min,
        angle_increment: msg.angle_increment * LASER_STEP,
        ranges:          sampledRanges
      }
    });
  });
}

// ─────────────────────────────────────────────────────────────────────────────
function subscribePlannedPath() {
  const planTopic = new ROSLIB.Topic({
    ros,
    name: '/move_base/NavfnROS/plan',
    messageType: 'nav_msgs/Path',
    throttle_rate: 500  // รับสูงสุด 2Hz
  });

  planTopic.subscribe((message) => {
    if (!isFresh(message.header?.stamp)) return; // ทิ้ง path เก่าที่สะสมระหว่างเน็ตสะดุด
    const pathPoints = message.poses.map(p => ({
      x: p.pose.position.x,
      y: p.pose.position.y
    }));
    parentPort.postMessage({ type: 'planned-path', data: pathPoints });
  });
}

// ─────────────────────────────────────────────────────────────────────────────
function subscribeMoveBaseResult() {
  if (!ros || !ros.isConnected) return;

  const resultTopic = new ROSLIB.Topic({
    ros,
    name: '/move_base/result',
    messageType: 'move_base_msgs/MoveBaseActionResult'
  });

  const STATUS_MAP = {
    0: 'PENDING',
    1: 'ACTIVE',
    2: 'PREEMPTED',
    3: 'SUCCEEDED',
    4: 'ABORTED',
    5: 'REJECTED',
    6: 'PREEMPTING',
    7: 'RECALLING',
    8: 'RECALLED',
    9: 'LOST'
  };

  resultTopic.subscribe((msg) => {
    if (!msg.status) return;
    const status = STATUS_MAP[msg.status.status] ?? 'UNKNOWN';
    const result = { status, text: msg.status.text || '' };

    if (!['UNKNOWN', 'ACTIVE', 'PENDING'].includes(status)) {
      parentPort.postMessage({ type: 'goal-result', data: result });
    }
  });
}

// ─────────────────────────────────────────────────────────────────────────────
function subscribeRobotStatus() {
  if (!ros || !ros.isConnected) return;

  const statusTopic = new ROSLIB.Topic({
    ros,
    name: '/robot/status',
    messageType: 'std_msgs/String',
    throttle_rate: 500
  });

  console.log('[Server] Subscribing to Robot Status: /robot/status');
  statusTopic.subscribe((msg) => {
    parentPort.postMessage({ type: 'robot-status-update', data: msg.data });
  });
}

// ─────────────────────────────────────────────────────────────────────────────
// Service calls
function callListMapsService() {
  if (!ros || !ros.isConnected) {
    parentPort.postMessage({ type: 'map-list', data: [], error: 'ROSBridge not connected' });
    return;
  }
  const service = new ROSLIB.Service({ ros, name: '/map_manager/list_maps', serviceType: 'ptR1_navigation/ListMaps' });
  service.callService(new ROSLIB.ServiceRequest({}), (result) => {
    parentPort.postMessage({ type: 'map-list', data: result.names });
  }, (err) => {
    console.error('❌ list_maps failed:', err);
    parentPort.postMessage({ type: 'map-list', data: [], error: err.toString() });
  });
}

function callSelectNavMapService(mapName) {
  if (!ros || !ros.isConnected) {
    parentPort.postMessage({ type: 'select-map-response', data: { success: false, message: 'ROSBridge not connected' } });
    return;
  }
  const service = new ROSLIB.Service({ ros, name: '/map_manager/select_nav_map', serviceType: 'ptR1_navigation/SelectNavMap' });
  service.callService(new ROSLIB.ServiceRequest({ name: mapName }), (result) => {
    parentPort.postMessage({ type: 'select-map-response', data: { ...result, name: mapName } });
  }, (err) => {
    console.error('❌ select_nav_map failed:', err);
    parentPort.postMessage({ type: 'select-map-response', data: { success: false, message: err.toString(), name: mapName } });
  });
}

function callStopNavService(shouldSavePose) {
  if (!ros || !ros.isConnected) { console.log('❌ ROS not connected.'); return; }
  const service = new ROSLIB.Service({ ros, name: '/nav/stop', serviceType: 'ptR1_navigation/StopAMCL' });
  service.callService(new ROSLIB.ServiceRequest({ save_pose: shouldSavePose || true }), (result) => {
    console.log('Navigation Stopped. Result:', result);
    parentPort.postMessage({ type: 'operation-result', data: { success: result.success, message: result.message } });
  }, (err) => { console.error('❌ /nav/stop failed:', err); });
}

function requestMapFileAsBase64(mapName) {
  const service = new ROSLIB.Service({ ros, name: '/map_manager/get_map_file', serviceType: 'ptR1_navigation/GetMapFile' });
  service.callService(new ROSLIB.ServiceRequest({ name: mapName }), (res) => {
    if (res.success) {
      parentPort.postMessage({ type: 'map-data', data: { name: mapName, base64: res.image_data_base64, yaml: res.yaml_data } });
    } else {
      console.warn(`❌ Map fetch failed: ${res.message}`);
    }
  });
}

function callSaveEditedMapService(newName, base64Data, yamlContent) {
  if (!ros || !ros.isConnected) {
    parentPort.postMessage({ type: 'map-save-edited', data: { success: false, message: 'ROS disconnected', name: newName } });
    return;
  }
  const service = new ROSLIB.Service({ ros, name: '/map_manager/save_edited_map', serviceType: 'ptR1_navigation/SaveEditedMap' });
  service.callService(new ROSLIB.ServiceRequest({ map_name: newName, base64_image: base64Data, yaml_content: yamlContent }), (result) => {
    parentPort.postMessage({ type: 'map-save-edited', data: { success: result.success, message: result.message, name: newName } });
  }, (err) => {
    parentPort.postMessage({ type: 'map-save-edited', data: { success: false, message: 'Service Error: ' + err.toString(), name: newName } });
  });
}

function callSaveMapService(mapName) {
  if (!ros || !ros.isConnected) {
    parentPort.postMessage({ type: 'map-save', data: { success: false, message: 'ROSBridge not connected', name: mapName } });
    return;
  }
  const service = new ROSLIB.Service({ ros, name: '/map_manager/save_map', serviceType: 'ptR1_navigation/SaveMap' });
  service.callService(new ROSLIB.ServiceRequest({ name: mapName }), (result) => {
    parentPort.postMessage({ type: 'map-save', data: { ...result, name: mapName } });
  }, (err) => {
    console.error('❌ save_map failed:', err);
    parentPort.postMessage({ type: 'map-save', data: { success: false, message: err.toString(), name: mapName } });
  });
}

function callStopSLAMService() {
  const service = new ROSLIB.Service({ ros, name: '/nav/stop', serviceType: 'ptR1_navigation/StopAMCL' });
  service.callService(new ROSLIB.ServiceRequest({}), (res) => {
    parentPort.postMessage({ type: 'slam-stop-result', data: { success: res.success, message: res.message } });
  }, (err) => {
    parentPort.postMessage({ type: 'slam-stop-result', data: { success: false, message: err.toString() } });
  });
}

function callStartSLAMService() {
  if (!ros || !ros.isConnected) {
    parentPort.postMessage({ type: 'slam-result', data: { success: false, message: 'ROSBridge not connected' } });
    return;
  }
  console.log('Server: Stopping Navigation before Starting SLAM...');
  const stopNavService = new ROSLIB.Service({ ros, name: '/nav/stop', serviceType: 'ptR1_navigation/StopAMCL' });
  stopNavService.callService(new ROSLIB.ServiceRequest({}),
    ()    => { console.log('Server: Navigation stopped. Starting SLAM...'); executeStartSLAM(); },
    (err) => { console.warn('Server: Could not stop navigation, trying SLAM anyway...', err); executeStartSLAM(); }
  );
}

function executeStartSLAM() {
  const service = new ROSLIB.Service({ ros, name: '/map_manager/start_slam', serviceType: 'ptR1_navigation/StartSLAM' });
  service.callService(new ROSLIB.ServiceRequest({}), (res) => {
    parentPort.postMessage({ type: 'slam-result', data: { success: res.success, message: res.message } });
  }, (err) => {
    parentPort.postMessage({ type: 'slam-result', data: { success: false, message: err.toString() } });
  });
}

// ── Cached publish: /move_base_simple/goal ────────────────────────────────────
function sendSingleGoalToMoveBase(data) {
  if (!ros || !ros.isConnected) return;
  if (!_goalTopic) {
    _goalTopic = new ROSLIB.Topic({ ros, name: '/move_base_simple/goal', messageType: 'geometry_msgs/PoseStamped' });
  }
  const msg = new ROSLIB.Message({
    header: { frame_id: 'map' },
    pose: { position: data.pose.position, orientation: data.pose.orientation }
  });
  console.log(`📍 Sending goal to (${data.pose.position.x.toFixed(2)}, ${data.pose.position.y.toFixed(2)})`);
  _goalTopic.publish(msg);
}

// ── Cached publish: /initialpose ─────────────────────────────────────────────
function publishInitialPose(pose) {
  if (!ros || !ros.isConnected) {
    console.error('Server : Cannot send initial pose: ROSBridge is not connected.');
    return;
  }
  if (!_initialPoseTopic) {
    _initialPoseTopic = new ROSLIB.Topic({ ros, name: '/initialpose', messageType: 'geometry_msgs/PoseWithCovarianceStamped' });
  }
  const message = new ROSLIB.Message({
    header: { frame_id: 'map' },
    pose: {
      pose: {
        position:    { x: pose.position.x, y: pose.position.y, z: 0 },
        orientation: pose.orientation
      },
      covariance: [
        0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.05
      ]
    }
  });
  console.log('Server : Publishing /initialpose');
  _initialPoseTopic.publish(message);
}

// ─────────────────────────────────────────────────────────────────────────────
function callStartStreamService() {
  if (!ros || !ros.isConnected) {
    parentPort.postMessage({ type: 'startStreamResponse', success: false, message: 'ROS is not connected.' });
    return;
  }
  const service = new ROSLIB.Service({ ros, name: '/stream_manager/start', serviceType: 'std_srvs/Trigger' });
  service.callService(new ROSLIB.ServiceRequest({}), (result) => {
    console.log('Server : Start Stream Result:', result);
    parentPort.postMessage({ type: 'startStreamResponse', success: result.success, message: result.message });
  }, (err) => {
    console.error('Server : Start Stream Error:', err);
    parentPort.postMessage({ type: 'startStreamResponse', success: false, message: err.toString() });
  });
}

function callStopStreamService() {
  if (!ros || !ros.isConnected) {
    parentPort.postMessage({ type: 'stopStreamResponse', success: true });
    return;
  }
  const service = new ROSLIB.Service({ ros, name: '/stream_manager/stop', serviceType: 'std_srvs/Trigger' });
  service.callService(new ROSLIB.ServiceRequest({}), (result) => {
    console.log('Server : Stop Stream Result:', result);
    parentPort.postMessage({ type: 'stopStreamResponse', success: result.success });
  });
}

function callDeleteMapService(mapName) {
  if (!ros || !ros.isConnected) {
    parentPort.postMessage({ type: 'map-delete-result', data: { success: false, message: 'ROSBridge not connected' } });
    return;
  }
  const service = new ROSLIB.Service({ ros, name: '/map_manager/delete_map', serviceType: 'ptR1_navigation/DeleteMap' });
  service.callService(new ROSLIB.ServiceRequest({ name: mapName }), (result) => {
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
  const service = new ROSLIB.Service({ ros, name: '/map_manager/reset_slam', serviceType: 'ptR1_navigation/ResetSLAM' });
  service.callService(new ROSLIB.ServiceRequest({}), (result) => {
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
  const service = new ROSLIB.Service({ ros, name: '/nav/start_patrol', serviceType: 'ptR1_navigation/StartPatrol' });
  service.callService(new ROSLIB.ServiceRequest({ goals, loop }), (result) => {
    parentPort.postMessage({ type: 'patrol-start-result', data: result });
  }, (err) => {
    parentPort.postMessage({ type: 'patrol-start-result', data: { success: false, message: err.toString() } });
  });
}

function callPausePatrolService() {
  if (!ros || !ros.isConnected) return;
  const service = new ROSLIB.Service({ ros, name: '/nav/pause_patrol', serviceType: 'ptR1_navigation/PausePatrol' });
  service.callService(new ROSLIB.ServiceRequest({}), (result) => {
    parentPort.postMessage({ type: 'patrol-pause-result', data: result });
  });
}

function callResumePatrolService() {
  if (!ros || !ros.isConnected) return;
  const service = new ROSLIB.Service({ ros, name: '/nav/resume_patrol', serviceType: 'ptR1_navigation/ResumePatrol' });
  service.callService(new ROSLIB.ServiceRequest({}), (result) => {
    parentPort.postMessage({ type: 'patrol-resume-result', data: result });
  });
}

function callStopPatrolService() {
  if (!ros || !ros.isConnected) return;
  const service = new ROSLIB.Service({ ros, name: '/nav/stop_patrol', serviceType: 'ptR1_navigation/StopPatrol' });
  service.callService(new ROSLIB.ServiceRequest({}), (result) => {
    parentPort.postMessage({ type: 'patrol-stop-result', data: result });
  });
}

function callStartNavService(restorePose) {
  if (!ros || !ros.isConnected) {
    parentPort.postMessage({ type: 'nav-start-response', data: { success: false, message: 'ROS not connected' } });
    return;
  }
  const service = new ROSLIB.Service({ ros, name: '/nav/start', serviceType: 'ptR1_navigation/StartAMCL' });
  console.log(`🚀 Calling /nav/start (restore=${restorePose})...`);
  service.callService(new ROSLIB.ServiceRequest({ restore_pose: restorePose }), (result) => {
    parentPort.postMessage({ type: 'nav-start-response', data: { success: result.success, message: result.message } });
  }, (err) => {
    parentPort.postMessage({ type: 'nav-start-response', data: { success: false, message: `Service Error: ${err}` } });
  });
}

function subscribePatrolStatus() {
  if (!ros || !ros.isConnected) return;
  const topic = new ROSLIB.Topic({ ros, name: '/nav/status', messageType: 'std_msgs/String' });
  console.log('[Server] Subscribing to Patrol Status: /nav/status');
  topic.subscribe((msg) => {
    parentPort.postMessage({ type: 'patrol-status', data: msg.data });
  });
}

function callHomeService(serviceName, mapName, actionLabel) {
  if (!ros || !ros.isConnected) {
    parentPort.postMessage({ type: 'home-result', data: { success: false, message: 'ROS not connected', action: actionLabel } });
    return;
  }
  const service = new ROSLIB.Service({ ros, name: serviceName, serviceType: 'ptR1_navigation/SaveMap' });
  service.callService(new ROSLIB.ServiceRequest({ name: mapName }), (result) => {
    console.log(`Server: ${actionLabel} Success`);
    if (actionLabel === 'Init Home') {
      parentPort.postMessage({ type: 'nav-init-home-response', data: { success: result.success, message: result.message } });
    }
    parentPort.postMessage({ type: 'home-result', data: { success: result.success, message: result.message, action: actionLabel } });
  }, (err) => {
    console.error(`Server: ${actionLabel} Failed`, err);
    if (actionLabel === 'Init Home') {
      parentPort.postMessage({ type: 'nav-init-home-response', data: { success: false, message: err.toString() } });
    }
    parentPort.postMessage({ type: 'home-result', data: { success: false, message: err.toString(), action: actionLabel } });
  });
}

// ─────────────────────────────────────────────────────────────────────────────
// TF — ใช้ threshold เดียวกับ pose เพื่อลด message
function subscribeTF() {
  if (!ros || !ros.isConnected) return;
  console.log('Server : Initializing TF Client...');

  tfClient = new ROSLIB.TFClient({
    ros,
    fixedFrame: 'map',
    angularThres: POSE_MIN_ANGLE,
    transThres:   POSE_MIN_DIST,
    rate: 10.0
  });

  tfClient.subscribe('base_link', (tf) => {
    parentPort.postMessage({
      type: 'tf-update',
      data: { translation: tf.translation, rotation: tf.rotation }
    });
  });
}

// ── Cached publish: /robot/cmdvel_manual ─────────────────────────────────────
function publishTwist(data) {
  if (!ros || !ros.isConnected) return;
  if (!_twistTopic) {
    _twistTopic = new ROSLIB.Topic({ ros, name: '/robot/cmdvel_manual', messageType: 'geometry_msgs/Twist' });
  }
  _twistTopic.publish(new ROSLIB.Message(data));
}

// ── Cached publish: /camera/tilt ─────────────────────────────────────────────
function publishServoTiltAngle(angle) {
  if (!ros || !ros.isConnected) return;
  if (!_servoTiltTopic) {
    _servoTiltTopic = new ROSLIB.Topic({ ros, name: '/camera/tilt', messageType: 'std_msgs/Int16' });
  }
  _servoTiltTopic.publish(new ROSLIB.Message({ data: angle }));
}

// ── Cached publish: /camera/pan ──────────────────────────────────────────────
function publishServoPanAngle(angle) {
  if (!ros || !ros.isConnected) return;
  if (!_servoPanTopic) {
    _servoPanTopic = new ROSLIB.Topic({ ros, name: '/camera/pan', messageType: 'std_msgs/Int16' });
  }
  _servoPanTopic.publish(new ROSLIB.Message({ data: angle }));
}


function subscribeSystemProfile() {
  if (!ros || !ros.isConnected) return;
  const topic = new ROSLIB.Topic({ ros, name: '/pi/system_profile', messageType: 'std_msgs/String', throttle_rate: 1000 });
  topic.subscribe((msg) => {
    try {
      parentPort.postMessage({ type: 'system-profile-update', data: JSON.parse(msg.data) });
    } catch (e) {
      console.error('Server: Error parsing system profile JSON', e);
    }
  });
}

function callUpdateDetectionService(settings) {
  if (!ros || !ros.isConnected) {
    parentPort.postMessage({ type: 'detection-update-result', data: { success: false, message: 'ROSBridge not connected' } });
    return;
  }
  const service = new ROSLIB.Service({ ros, name: '/stream_manager/update_detection', serviceType: 'ptR1_navigation/UpdateDetection' });
  service.callService(new ROSLIB.ServiceRequest({
    mode:       settings.mode,
    time_start: settings.time_start,
    time_end:   settings.time_end,
    classes:    settings.classes,
    enabled:    settings.enabled
  }), (result) => {
    parentPort.postMessage({ type: 'detection-update-result', data: { success: result.success, message: result.message } });
  }, (err) => {
    parentPort.postMessage({ type: 'detection-update-result', data: { success: false, message: err.toString() } });
  });
}

function subscribeDetectionAlert() {
  if (!ros || !ros.isConnected) return;
  const topic = new ROSLIB.Topic({ ros, name: '/stream_manager/alert', messageType: 'std_msgs/String' });
  topic.subscribe((msg) => {
    try {
      parentPort.postMessage({ type: 'detection-alert', data: JSON.parse(msg.data) });
    } catch (e) {
      console.error('[Server] Failed to parse alert:', e);
    }
  });
}

parentPort.postMessage({ type: 'log', data: 'Worker Initialized' });