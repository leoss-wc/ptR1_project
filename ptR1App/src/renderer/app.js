console.log('👷 app.js started');

import { initRelayButtons } from './modules/relayControl.js';
import { CanvasRecorder } from './modules/recorder.js';

import { initStaticMap } from './modules/mapStatic.js';
import { renderCanvas as renderStaticMapCanvas } from './modules/mapStatic.js';


import { renderDashboardMap, initHomeMap} from './modules/mapHome.js';

import { WebRTCPlayer } from './modules/webrtc-player.js';
import { setupVideoPlayer } from './modules/videoPlayer.js';

import { updateRobotPose } from './modules/robotState.js';
import { setPlannedPath } from './modules/planState.js';
import * as patrolState from './modules/patrolState.js';

import * as patrol from './modules/patrol.js';

import { FrameProcessor } from './modules/FrameProcessor.js';
import { OverlayCanvas } from './modules/OverlayCanvas.js';

import {resetLiveMapView,processLiveMapData, initLiveMap, drawLiveMap, updateLiveRobotPose } from './modules/mapLive.js';
import * as mapView from './modules/mapView.js'; // Import ทุกอย่างจาก mapView


let recorder = null;
let rtcPlayer = null;
let allRobotProfiles = []; //ตัวแปรสำหรับเก็บโปรไฟล์ทั้งหมดที่โหลดมา
let selectedProfileName = null; //ตัวแปรสำหรับเก็บชื่อโปรไฟล์ที่กำลังเลือก
let frameProcessor = null;
let yoloOverlay = null;
let isFirstMapReceived = false; // ตัวแปรเพื่อตรวจสอบว่าได้รับข้อมูลแผนที่ครั้งแรกหรือยัง
let lastFrameTime = 0;
const targetFPS = 1; //live map  Frame Rate 
const fpsInterval = 1000 / targetFPS;
let liveMapRenderId = null;

let mockMoveInterval = null;
function startMockPathTest() {
  if (mockMoveInterval) {
    clearInterval(mockMoveInterval);
    mockMoveInterval = null;
    console.log("🛑 Stopped path test.");
    return;
  }
  console.log("🚀 Starting path test...");

  // 1. กำหนดจุดเริ่มต้น, เป้าหมาย, และสร้างเส้นทางจำลอง (เส้นตรง)
  const startPos = { x: -2, y: -3, z: 0 };
  const goalPos = { x: 5, y: 2, z: 0 };
  
  let mockPath = [];
  const steps = 50; // จำนวนก้าว
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    mockPath.push({
      x: startPos.x + t * (goalPos.x - startPos.x),
      y: startPos.y + t * (goalPos.y - startPos.y)
    });
  }

  // 2. ตั้งค่า State เริ่มต้น
  updateRobotPose(startPos, { x: 0, y: 0, z: 0, w: 1 }); // วางหุ่นยนต์ที่จุดเริ่มต้น
  patrolState.setGoalPoint(goalPos); // กำหนดเป้าหมาย (จุดสีแดง)
  
  let currentStep = 0;

  mockMoveInterval = setInterval(() => {
    if (currentStep >= mockPath.length) {
      clearInterval(mockMoveInterval);
      mockMoveInterval = null;
      console.log("🏁 Path test finished.");
      setPlannedPath([]); // เคลียร์เส้นทางเมื่อถึงเป้าหมาย
      renderDashboardMap();
      return;
    }

    // 3. อัปเดต State ในแต่ละเฟรม
    const currentPos = mockPath[currentStep];
    const remainingPath = mockPath.slice(currentStep);

    updateRobotPose(currentPos, { x: 0, y: 0, z: 0, w: 1 }); // อัปเดตตำแหน่งหุ่นยนต์
    setPlannedPath(remainingPath); // อัปเดตเส้นทางที่เหลือ

    // 4. สั่งวาดใหม่
    renderDashboardMap();

    currentStep++;
  }, 150); // อัปเดตทุกๆ 150ms
}

document.addEventListener('DOMContentLoaded', async() => {
  console.log("app: DOMContentLoaded fired!");
  // Video Player view setup
  setupVideoPlayer();
  //ตั้งค่าการสลับ View ผ่าน Sidebar
  document.querySelectorAll('.sidebar-item').forEach(item => {
    item.addEventListener('click', () => switchView(item.dataset.view));
  });
  const pwmSlider = document.getElementById('pwm-slider');
  const pwmValueLabel = document.getElementById('pwm-value-label');
  if (pwmSlider && pwmValueLabel) {
    pwmSlider.addEventListener('input', () => pwmValueLabel.textContent = pwmSlider.value);
  }
  // Map Mode Toggle (Static/Live)
  document.getElementById('btn-static-map').addEventListener('click', () => {
    document.getElementById('staticMapCanvas').classList.remove('hidden');
    document.getElementById('liveMapCanvas').classList.add('hidden');
    document.getElementById('btn-static-map').classList.add('active');
    document.getElementById('btn-live-map').classList.remove('active');
    document.getElementById('static-control-box').classList.remove('hidden');
    document.getElementById('live-control-box').classList.add('hidden');
    document.getElementById('patrol-status-label').classList.remove('hidden');
    document.querySelector('.canvas-controls').classList.remove('hidden');
    stopLiveMapRender();

  });
  document.getElementById('btn-live-map').addEventListener('click', () => {
    document.getElementById('staticMapCanvas').classList.add('hidden');
    document.getElementById('liveMapCanvas').classList.remove('hidden');
    document.getElementById('btn-static-map').classList.remove('active');
    document.getElementById('btn-live-map').classList.add('active');
    document.getElementById('static-control-box').classList.add('hidden');
    document.getElementById('live-control-box').classList.remove('hidden');
    document.getElementById('patrol-status-label').classList.add('hidden');
    document.querySelector('.canvas-controls').classList.add('hidden');
    startLiveMapRender();
  });
  const mapWrapper = document.querySelector('.map-wrapper');
    if (mapWrapper) {
      // ส่ง container และ callback ของทั้งสอง map ไปให้ mapView จัดการ
      mapView.initMapViewController(
        mapWrapper,
        renderStaticMapCanvas, // Callback สำหรับ Static Map
        drawLiveMap           // Callback สำหรับ Live Map
      );
    }
    document.getElementById('reset-live-view-btn').addEventListener('click', () => {
    resetLiveMapView();
    });

  requestAnimationFrame(renderLoop);

  switchView('home')

  document.getElementById('save-path-btn').addEventListener('click', patrol.saveDrawnPath);
  document.getElementById('start-patrol-btn').addEventListener('click', patrol.startPatrol);
  document.getElementById('pause-patrol-btn').addEventListener('click', patrol.pausePatrol);
  document.getElementById('resume-patrol-btn').addEventListener('click', patrol.resumePatrol);
  document.getElementById('stop-patrol-btn').addEventListener('click', patrol.stopPatrol);
  
  patrol.initPatrolManager();

  document.getElementById('loop-patrol-checkbox').addEventListener('change', (event) => {
    patrolState.setLooping(event.target.checked);
  });

  // Relay
  initRelayButtons();

  // Recorder 
  const canvas = document.getElementById('capture-canvas');
  const startBtn = document.getElementById('start-record');
  const stopBtn = document.getElementById('stop-record');

  const testButton = document.getElementById('test-robot-move-btn');
  if (testButton) {
    testButton.addEventListener('click', startMockPathTest);
  }
  // --- ส่วนตรวจสอบเชื่อมต่อ ROS Bridge ---
  const rosStatusEl = document.getElementById('home-ros-status');

    window.electronAPI.onConnectionStatus((status) => {
        console.log('ROS Connection Status Update:', status);
        rosStatusEl.textContent = status.message;
        
        // อัปเดต Class เพื่อเปลี่ยนสี
        rosStatusEl.className = ''; // เคลียร์ class เก่า
        if (status.connected) {
            rosStatusEl.classList.add('status-connected');
        } else if (status.connecting) {
            rosStatusEl.classList.add('status-connecting');
        } else {
            rosStatusEl.classList.add('status-disconnected');
        }
    });

  recorder = new CanvasRecorder(canvas, {
    fps: 30,
    segmentMs: 10 * 60 * 1000 // หรือเปลี่ยนเป็น 10 * 1000 สำหรับ dev
  });

  startBtn.addEventListener('click', () => {
    recorder.start();
    startBtn.disabled = true;
    stopBtn.disabled = false;
  });

  stopBtn.addEventListener('click', () => {
    recorder.stop();
    startBtn.disabled = false;
    stopBtn.disabled = true;
  });

  const keyboardToggle = document.getElementById('keyboard-toggle');
  const pwmInput = document.getElementById('pwm-slider');
  const cmdDropdown = document.getElementById('cmd-dropdown');
  const sendSelectedCmdButton = document.getElementById('send-selected-cmd-button');
  const cmdInput = document.getElementById('cmd-input');
  const sendCustomCmdButton = document.getElementById('send-custom-cmd-button');
  const modeLabel = document.getElementById('mode-label');
  const connectButton = document.getElementById('connectButton');

  // --- ส่วนจัดการ Settings ---
  document.getElementById('robot-profile-select').addEventListener('change', handleProfileSelection);
  document.getElementById('add-profile-btn').addEventListener('click', addNewProfile);
  document.getElementById('save-profile-btn').addEventListener('click', saveProfile);
  document.getElementById('delete-profile-btn').addEventListener('click', deleteProfile);
  document.getElementById('connect-all-btn').addEventListener('click', connectUsingCurrentProfile);
  document.getElementById('connect-video-btn').addEventListener('click', connectVideoPlayer);
    
  // โหลดโปรไฟล์ทั้งหมดเมื่อแอปเริ่มทำงาน
  await loadAndDisplayProfiles();
    

  connectButton.addEventListener('click', () => {
    //window.electronAPI.connectROSBridge(ip);
    connectUsingCurrentProfile();
    console.log(`Connectting`);
  });

  // --- ส่วนจัดการ Video Stream และ YOLO Overlay ---
  document.getElementById('start-stream-btn').addEventListener('click', () => {
      console.log("Requesting to start FFmpeg stream...");
      window.electronAPI.startFFmpegStream();
  });
  document.getElementById('stop-stream-btn').addEventListener('click', () => {
      console.log("Requesting to stop FFmpeg stream...");
      window.electronAPI.stopFFmpegStream();
  });
  //Listener เพื่อรับ Feedback สถานะการสั่งงาน Stream
  window.electronAPI.onStreamStatus((result) => {
      console.log("Stream Status Update:", result);
      alert(`Stream command status: ${result.success ? 'Success' : 'Failed'}\nMessage: ${result.message}`);
  });


  // --- ส่วนจัดการ SLAM ---
  const startSlamBtn = document.getElementById('start-slam-btn');
  const stopSlamBtn = document.getElementById('stop-slam-btn');
  const saveMapBtn = document.getElementById('save-map-btn');
  const mapNameInput = document.getElementById('map-name-input');
  const slamResultLabel = document.getElementById('slam-result-label');
  // กดปุ่ม Start SLAM
  startSlamBtn.addEventListener('click', () => {
    console.log('Requesting to start SLAM...');
    slamResultLabel.textContent = 'Starting SLAM...';
    slamResultLabel.style.color = 'yellow';
    window.electronAPI.startSLAM();
  });
  // กดปุ่ม Stop SLAM
  stopSlamBtn.addEventListener('click', () => {
    console.log('Requesting to stop SLAM...');
    slamResultLabel.textContent = 'Stopping SLAM...';
    slamResultLabel.style.color = 'yellow';
    window.electronAPI.stopSLAM();
  });
  // กดปุ่ม Save Map
  saveMapBtn.addEventListener('click', () => {
    const mapName = mapNameInput.value.trim();
    if (!mapName) {
      alert('Please enter a map name before saving.');
      return;
    }
    console.log(`Requesting to save map as "${mapName}"...`);
    slamResultLabel.textContent = `Saving map: ${mapName}...`;
    slamResultLabel.style.color = 'yellow';
    window.electronAPI.saveMap(mapName);
  });


  if (!keyboardToggle || !pwmInput || !cmdDropdown || !sendSelectedCmdButton || !cmdInput || !sendCustomCmdButton || !modeLabel) {
    console.error("❌ UI elements not found! Check HTML structure.");
    return;
  }
  // ✅ ฟังก์ชันส่ง UInt32 command
  const sendUInt32Command = (command) => {
    if (isNaN(command) || command < 0 || command > 0xFFFFFFFF) {
      console.error('❌ Invalid UInt32 value');
      return;
    }
    console.log(`Sending UInt32 Command: ${command}`);
    window.robotControl.sendCommand(command);
  };
  // ✅ กดปุ่มส่งจาก Dropdown
  sendSelectedCmdButton.addEventListener('click', () => {
    const selectedCmd = parseInt(cmdDropdown.value);
    sendUInt32Command(selectedCmd);
  });

  // ✅ กดปุ่มส่งจาก Text Input
  sendCustomCmdButton.addEventListener('click', () => {
    const customCmd = parseInt(cmdInput.value, 16);
    sendUInt32Command(customCmd);
  });

  keyboardToggle.addEventListener('change', (event) => {
    const isOn = event.target.checked;
    modeLabel.textContent = isOn ? 'MANUAL ON' : 'MANUAL OFF';

    console.log(isOn ? '🛠 Switched to MANUAL ON' : '🛑 Switched to MANUAL OFF');
    window.electronAPI.setManualMode(isOn);
  });

  if (window.electronAPI?.onPowerUpdate) {
  window.electronAPI.onPowerUpdate((data) => {
  //console.log("Power data received:", data);
  const { voltage, current, percent } = data;

  document.getElementById('voltage').textContent = `${voltage} V`;
  //document.getElementById('current').textContent = `${current} A`;
  document.getElementById('percent').textContent = `${percent} %`;

  // เปลี่ยนสีถ้าแบตต่ำกว่า 20%
  const percentEl = document.getElementById('percent');
  percentEl.style.color = parseFloat(percent) < 20 ? 'red' : 'white';
  });
  } else {
  console.warn("⚠️ electronAPI.onPowerUpdate is undefined.");
  }
});

// ✅ ฟังก์ชันส่ง Drive command เฉพาะตอน MANUAL ON

const pressedKeys = new Set();
const intervalMap = new Map();
const pwmMap = new Map();
const pwmStep = 5;
const pwmInitial = 40;

const sendKeyDrive = (event) => {
  if (!event || !event.code) return;

  const keyboardToggle = document.getElementById('keyboard-toggle');
  let pwmInput = document.getElementById('pwm-slider');
  const modeLabel = document.getElementById('mode-label');

  if (!keyboardToggle || !pwmInput || !modeLabel) return;

  if (modeLabel.textContent.trim().toUpperCase() !== 'MANUAL ON') return;
  const pwmInputValue = parseInt(pwmInput.value);
  const pwmMax = (!isNaN(pwmInputValue) && pwmInputValue > 0) ? pwmInputValue : 255;

  const hadPwm = pwmMap.has(event.code);
  let pwm = hadPwm ? pwmMap.get(event.code) : pwmInitial;
  let command;
  //let pwm = pwmMap.has(event.code) ? pwmMap.get(event.code) : pwmInitial;
  pwm = Math.min(pwm + pwmStep, pwmMax);
  pwmMap.set(event.code, pwm);  // เก็บ pwm ล่าสุด

  switch (event.code) {
    case 'KeyW': command = 0x0100 + pwm; break; // Forward
    case 'KeyS': command = 0x0400 + pwm; break; // Backward
    case 'KeyA': command = 0x0200 + pwm; break; // Left
    case 'KeyD': command = 0x0300 + pwm; break; // Right
    case 'KeyR': command = 0x0500 + pwm; break; // turn left
    case 'KeyF': command = 0x0600 + pwm; break; // turn right
    case 'KeyQ': command = 0x0700 + pwm; break; // forward left
    case 'KeyE': command = 0x0800 + pwm; break; // forward right
    default: return;
  }
  command = command & 0xFFFF;
  console.log(`Key pressed: ${event.code} -> Command: ${command.toString(16).toUpperCase()}, PWM: ${pwm}`);
  window.robotControl.sendKeyCommand(command);
};
document.addEventListener('keydown', (event) => {
  const code = event.code;

  if (pressedKeys.has(code)) return;
  pressedKeys.add(code);

  const isServoKey = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(code);

  if (!isServoKey) {
    pwmMap.set(code, pwmInitial);
  }

  const intervalId = setInterval(() => {
    if (isServoKey) {
      sendServoControl(event);
    } else {
      sendKeyDrive(event);
    }
  }, isServoKey ? 150 : 100);

  intervalMap.set(code, intervalId);
});
document.addEventListener('keyup', (event) => {
  const code = event.code;

  //console.log(`⏏️ Key released: ${code}, clearing interval and PWM`);
  //console.log('PWM before delete:', pwmMap.get(code));

  if (intervalMap.has(code)) {
    clearInterval(intervalMap.get(code));
    intervalMap.delete(code);
  }
  pwmMap.delete(code);
  pressedKeys.delete(code);
});

//ฟังก์ชันส่ง Servo command เฉพาะตอน MANUAL ON
const sendServoControl = (event) => {
  if (!event || !event.code) return;

  const keyboardToggle = document.getElementById('keyboard-toggle');
  const modeLabel = document.getElementById('mode-label');

  if (!keyboardToggle || !modeLabel) return;
  if (modeLabel.textContent.trim().toUpperCase() !== 'MANUAL ON') return;

  let command;

  switch (event.code) {
    case 'ArrowRight':    command = 0x01; break;
    case 'ArrowLeft':  command = 0x02; break;
    case 'ArrowUp':  command = 0x03; break;
    case 'ArrowDown': command = 0x04; break;
    default: return;
  }

  console.log(`Servo key: ${event.code} -> Command: ${command}`);
  window.robotControl.sendServoCommand(command);
};

window.electronAPI.onLiveMap((mapData) => {
  // 1. ประมวลผลและให้ mapLive.js เก็บ state
  processLiveMapData(mapData);
  
  // 2. สั่งวาด Canvas
  const liveMapCanvas = document.getElementById('liveMapCanvas');
  if (liveMapCanvas && !liveMapCanvas.classList.contains('hidden')) {
    if (!isFirstMapReceived) {
      resetLiveMapView();
      isFirstMapReceived = true;
    }
  }
});

window.electronAPI.onRobotPosSlam((poseData) => {
  if (poseData.position && poseData.orientation) {
    updateLiveRobotPose(poseData);
  }
});

window.electronAPI.onRobotPosAmcl((poseData) => {
  if (poseData.position && poseData.orientation) {
    updateRobotPose(poseData.position, poseData.orientation);
    renderDashboardMap();
  }
});

window.electronAPI.onPlannedPath((pathData) => {
  setPlannedPath(pathData);
});

function renderLoop(currentTime) {
  // Get the canvas element.
  const liveMapCanvas = document.getElementById('liveMapCanvas');

  // 🛑 SELF-STOPPING GUARD: If the canvas is hidden or doesn't exist,
  // stop the render loop immediately by not requesting the next frame.
  if (!liveMapCanvas || liveMapCanvas.classList.contains('hidden')) {
    liveMapRenderId = null; // Ensure the state reflects that the loop is stopped.
    return;
  }

  // Continue the loop by requesting the next animation frame.
  liveMapRenderId = requestAnimationFrame(renderLoop);
  
  // Throttle the drawing to the specified FPS.
  const elapsed = currentTime - lastFrameTime;
  if (elapsed > fpsInterval) {
    // Adjust lastFrameTime for more accurate throttling.
    lastFrameTime = currentTime - (elapsed % fpsInterval);
    
    // Draw the map. No need to check for visibility again.
    drawLiveMap(); 
  }
}

function stopLiveMapRender() {
  // ถ้ามีการเรนเดอร์อยู่ ให้หยุด
  if (liveMapRenderId) {
    console.log("Stopping Live Map render loop.");
    cancelAnimationFrame(liveMapRenderId);
    liveMapRenderId = null;
  }
}
function startLiveMapRender() {
  // Only start a new loop if one isn't already running.
  if (!liveMapRenderId) {
    console.log("Starting Live Map render loop.");
    // Initialize the timer to start throttling correctly from the first frame.
    lastFrameTime = performance.now();
    // Use requestAnimationFrame to start the loop smoothly.
    liveMapRenderId = requestAnimationFrame(renderLoop);
  }
}
async function loadAndDisplayProfiles() {
    const profileSelect = document.getElementById('robot-profile-select');
    profileSelect.innerHTML = '<option value="">-- Select a Profile --</option>'; // ค่าเริ่มต้น

    allRobotProfiles = await window.electronAPI.loadRobots();
    
    if (allRobotProfiles && allRobotProfiles.length > 0) {
        allRobotProfiles.forEach(profile => {
            const option = document.createElement('option');
            option.value = profile.name;
            option.textContent = profile.name;
            profileSelect.appendChild(option);
        });
    }

    // ลองโหลดโปรไฟล์ล่าสุดที่เคยใช้จาก localStorage
    const lastSelected = localStorage.getItem('lastSelectedProfile');

    if (lastSelected && allRobotProfiles.some(p => p.name === lastSelected)) {
        profileSelect.value = lastSelected;
        handleProfileSelection(); // เรียกใช้เพื่อแสดงผล
    } else {
         // ถ้าไม่มี ให้ซ่อนฟอร์มไว้ก่อน
        document.getElementById('profile-form-section').style.display = 'none';
    }
}

/**
 * เมื่อผู้ใช้เลือกโปรไฟล์จาก Dropdown
 */
function handleProfileSelection() {
    const profileSelect = document.getElementById('robot-profile-select');
    const formSection = document.getElementById('profile-form-section');
    selectedProfileName = profileSelect.value;
    
    if (!selectedProfileName) {
        formSection.style.display = 'none'; // ซ่อนฟอร์มถ้าไม่ได้เลือกอะไร
        return;
    }
    // ค้นหาข้อมูลโปรไฟล์ที่เลือก
    const profileData = allRobotProfiles.find(p => p.name === selectedProfileName);
    if (profileData) {
      // แสดงข้อมูลในฟอร์ม
        document.getElementById('profile-name').value = profileData.name || '';
        document.getElementById('profile-address').value = profileData.address || '';
        document.getElementById('profile-ros-port').value = profileData.rosPort || '9090';
        // ลบบรรทัดที่เกี่ยวกับ camera-port
        document.getElementById('profile-whep-port').value = profileData.whepPort || '8889';
        
        // ทำให้ช่องชื่อแก้ไขไม่ได้ (เพราะเป็น key)
        document.getElementById('profile-name').disabled = true; 
        document.getElementById('form-title').textContent = `Editing: ${profileData.name}`;
        formSection.style.display = 'block'; // แสดงฟอร์ม
        
        // บันทึกตัวเลือกล่าสุดไว้
        localStorage.setItem('lastSelectedProfile', selectedProfileName);
        console.log(`Saved "${selectedProfileName}" as the last used profile.`);
        updateHomePanel(profileData);
    }
    else {
        updateHomePanel(null);
    }
}

/**
 * อัปเดตข้อมูลโปรไฟล์ที่แสดงใน Panel ของหน้า Home
 * @param {object} profileData - ข้อมูลโปรไฟล์ที่กำลังใช้งาน
 */
function updateHomePanel(profileData) {
    const nameEl = document.getElementById('home-profile-name');
    const addressEl = document.getElementById('home-profile-address');

    if (profileData && profileData.name) {
        nameEl.textContent = profileData.name;
        addressEl.textContent = `${profileData.address}:${profileData.rosPort}`;
    } else {
        nameEl.textContent = 'None';
        addressEl.textContent = 'N/A';
    }
}
function addNewProfile() {
    const formSection = document.getElementById('profile-form-section');
    
    // เคลียร์ฟอร์มและตั้งค่าสำหรับโปรไฟล์ใหม่
    document.getElementById('profile-name').value = '';
    document.getElementById('profile-address').value = ''; // เคลียร์ Address

    // ตั้งค่า Port กลับไปเป็นค่าเริ่มต้น (user-friendly กว่าการเคลียร์เป็นค่าว่าง)
    document.getElementById('profile-ros-port').value = '9090';
    document.getElementById('profile-whep-port').value = '8889';

    // ทำให้ช่องชื่อโปรไฟล์แก้ไขได้และ focus ไปที่ช่องนั้น
    document.getElementById('profile-name').disabled = false; 
    document.getElementById('profile-name').focus();
    document.getElementById('form-title').textContent = '➕ Add New Profile';
    
    // ยกเลิกการเลือกโปรไฟล์เก่าใน Dropdown และ State
    selectedProfileName = null; 
    document.getElementById('robot-profile-select').value = '';
    formSection.style.display = 'block';

    // อัปเดต Home panel ให้แสดงว่าไม่มีโปรไฟล์ถูกเลือก
    updateHomePanel(null);
}

/**
 * บันทึกข้อมูลโปรไฟล์ (ทั้งใหม่และแก้ไข)
 */
async function saveProfile() {
    const statusEl = document.getElementById('settings-status');
    const newName = document.getElementById('profile-name').value.trim();

    if (!newName) {
        statusEl.textContent = '❌ Profile Name cannot be empty.';
        statusEl.style.color = 'red';
        return;
    }

    const updatedProfileData = {
        name: newName,
        address: document.getElementById('profile-address').value.trim(),
        rosPort: parseInt(document.getElementById('profile-ros-port').value, 10),
        whepPort: parseInt(document.getElementById('profile-whep-port').value, 10),
    };

    if (selectedProfileName) { // --- กรณีแก้ไข ---
        // หา index แล้วอัปเดตข้อมูล
        const index = allRobotProfiles.findIndex(p => p.name === selectedProfileName);
        if (index > -1) {
            allRobotProfiles[index] = updatedProfileData;
        }
    } else { // --- กรณีสร้างใหม่ ---
        // ตรวจสอบว่าชื่อซ้ำหรือไม่
        if (allRobotProfiles.some(p => p.name === newName)) {
            statusEl.textContent = '❌ Profile name already exists.';
            statusEl.style.color = 'red';
            return;
        }
        allRobotProfiles.push(updatedProfileData);
    }
    
    // บันทึกข้อมูลทั้งหมดลงไฟล์
    const result = await window.electronAPI.saveRobots(allRobotProfiles);
    if (result) {
        statusEl.textContent = '✅ Profile saved successfully!';
        statusEl.style.color = 'green';
        await loadAndDisplayProfiles(); // โหลดและแสดงผล Dropdown ใหม่
        // เลือกโปรไฟล์ที่เพิ่งบันทึกไป
        document.getElementById('robot-profile-select').value = newName;
        handleProfileSelection();
    } else {
        statusEl.textContent = '❌ Error saving profile.';
        statusEl.style.color = 'red';
    }
}

/**
 * ลบโปรไฟล์ที่เลือก
 */
async function deleteProfile() {
    if (!selectedProfileName) {
        alert("Please select a profile to delete.");
        return;
    }

    if (confirm(`Are you sure you want to delete the profile "${selectedProfileName}"?`)) {
        allRobotProfiles = allRobotProfiles.filter(p => p.name !== selectedProfileName);
        await window.electronAPI.saveRobots(allRobotProfiles);
        await loadAndDisplayProfiles(); // โหลดใหม่
        document.getElementById('settings-status').textContent = `🗑️ Profile "${selectedProfileName}" deleted.`;
    }
}
/**
 * เชื่อมต่อโดยใช้ข้อมูลจากฟอร์มปัจจุบัน
 */
function connectUsingCurrentProfile() {
    const statusEl = document.getElementById('settings-status');
    const address = document.getElementById('profile-address').value;
    const rosPort = document.getElementById('profile-ros-port').value;

    if (!address || !rosPort) {
        statusEl.textContent = '❌ Please fill in address and ROS port fields.';
        statusEl.style.color = 'red';
        return;
    }

    const rosIp = address;
    console.log(`🔌 Connecting to ROSBridge at ${rosIp}:${rosPort}`);
    window.electronAPI.connectROSBridge(rosIp);

    statusEl.textContent = `🚀 Attempting to connect to ROS using profile: ${selectedProfileName}`;
    statusEl.style.color = 'green';
}

function connectVideoPlayer() {
    const address = document.getElementById('profile-address').value;
    const whepPort = document.getElementById('profile-whep-port').value;
    if (!address || !whepPort) {
        alert("❌ Please fill in address and WHEP port fields.");
        return;
    }

    const whepUrl = `http://${address}:${whepPort}/live/whep`;
    const videoElement = document.getElementById('stream');
    const webrtcStatusElement = document.getElementById('rtc_status');
    
    // --- เชื่อมต่อ WebRTC Player ---
    if (rtcPlayer) rtcPlayer.disconnect();
    rtcPlayer = new WebRTCPlayer(whepUrl, videoElement, webrtcStatusElement);
    rtcPlayer.connect();

    // --- เริ่มต้น Frame Processor สำหรับ YOLO ---
    if (yoloOverlay) yoloOverlay.clear();
    yoloOverlay = new OverlayCanvas('yolo-overlay', videoElement);
    
    if (frameProcessor) frameProcessor.stop();
    frameProcessor = new FrameProcessor(videoElement, (detections) => {
        if (yoloOverlay) {
            yoloOverlay.drawDetections(detections);
        }
    });

    setTimeout(() => {
        if(yoloOverlay) yoloOverlay.resize();
        frameProcessor.start();
    }, 2000); // หน่วงเวลาเพื่อให้ WebRTC เริ่มทำงานก่อน
}

window.electronAPI.onSLAMStartResult((data) => {
  console.log('SLAM Start Result:', data);
  const slamResultLabel = document.getElementById('slam-result-label');
  slamResultLabel.textContent = data.message;
  slamResultLabel.style.color = data.success ? 'lime' : 'red';
});

// รับผลลัพธ์จากการ Stop SLAM
window.electronAPI.onSLAMStopResult((data) => {
  console.log('SLAM Stop Result:', data);
  const slamResultLabel = document.getElementById('slam-result-label');
  slamResultLabel.textContent = data.message;
  slamResultLabel.style.color = data.success ? 'lime' : 'red';
});

// รับผลลัพธ์จากการ Save Map
window.electronAPI.onMapSaveResult((result) => {
    console.log('Map Save Result:', result);
    const slamResultLabel = document.getElementById('slam-result-label');
    slamResultLabel.textContent = `Save '${result.name}': ${result.message}`;
    slamResultLabel.style.color = result.success ? 'lime' : 'red';

    // ถ้าสำเร็จ ให้ทำการ Sync แผนที่ใหม่ทันที
    if (result.success) {
      alert(`Map "${result.name}" saved successfully! Syncing maps...`);
      // อาจจะเรียกใช้ฟังก์ชัน sync maps ที่มีอยู่แล้ว
      document.getElementById('sync-maps-btn').click(); 
    }
});

function switchView(viewName) {
  // ซ่อนทุก View และเอา active ออกจาก sidebar
  document.querySelectorAll('.view').forEach(view => view.classList.add('hidden'));
  document.querySelectorAll('.sidebar-item').forEach(item => item.classList.remove('active'));

  // แสดง View และ Sidebar item ที่ต้องการ
  const activeView = document.getElementById(`view-${viewName}`);
  const activeSidebarItem = document.querySelector(`.sidebar-item[data-view="${viewName}"]`);
  if (activeView) activeView.classList.remove('hidden');
  if (activeSidebarItem) activeSidebarItem.classList.add('active');
  
  if (viewName === 'home') {
    const homeCanvas = document.getElementById('homeMapCanvas');
    if (homeCanvas) initHomeMap(homeCanvas);
  } else if (viewName === 'map') {
    initStaticMap();
    initLiveMap();
  }
}
