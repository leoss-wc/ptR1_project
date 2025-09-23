console.log('👷 app.js started');

import { initRelayButtons } from './modules/relayControl.js';
import { CanvasRecorder } from './modules/recorder.js';

import { initStaticMap } from './modules/mapStatic.js';

import { setupMapCanvas , setMapImage, renderDashboardMap} from './modules/mapHome.js';
import { activeMap } from './modules/mapState.js';

import { WebRTCPlayer } from './modules/webrtc-player.js';
import { setupVideoPlayer } from './modules/videoPlayer.js';

import { updateRobotPose } from './modules/robotState.js';
import { setPlannedPath } from './modules/planState.js';

import { setGoalPoint, isPatrolling, setPatrolPath } from './modules/patrolState.js';

let recorder = null;
let rtcPlayer = null;
let allRobotProfiles = []; //ตัวแปรสำหรับเก็บโปรไฟล์ทั้งหมดที่โหลดมา
let selectedProfileName = null; //ตัวแปรสำหรับเก็บชื่อโปรไฟล์ที่กำลังเลือก

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
  setGoalPoint(goalPos); // กำหนดเป้าหมาย (จุดสีแดง)
  
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

  const homeMapCanvas = document.getElementById('homeMapCanvas');
  if (!homeMapCanvas) return;
  const savedMapName = localStorage.getItem('activeMapName');
  if (savedMapName) {
    setupMapCanvas(homeMapCanvas); // เตรียม Canvas ให้พร้อม
    console.log("map canvas initialized.");
    console.log(`Found saved active map: ${savedMapName}. Loading...`);
    // เรียกใช้ API เพื่อดึงข้อมูลแผนที่ทั้งหมดด้วยชื่อ
    const mapData = await window.electronAPI.getMapDataByName(savedMapName);
    console.log("Data received from main process:", mapData); 
    
    if (mapData.success) {
      // อัปเดต State
      activeMap.name = mapData.name;
      activeMap.base64 = mapData.base64;
      activeMap.meta = mapData.meta;
      
      // สั่งวาดลง homeMapCanvas!
      await setMapImage(activeMap.base64);
      console.log(`✅ Automatically loaded and displayed '${savedMapName}' on dashboard.`);
    }
  }
   // Static Map
  initStaticMap();

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
    
    // โหลดโปรไฟล์ทั้งหมดเมื่อแอปเริ่มทำงาน
    await loadAndDisplayProfiles();
    
    // ... (ส่วนที่เหลือของ DOMContentLoaded) ...
    initRelayButtons();

  connectButton.addEventListener('click', () => {
    //window.electronAPI.connectROSBridge(ip);
    connectUsingCurrentProfile();
    console.log(`Connectting`);
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

// ✅ ฟังก์ชันส่ง Servo command เฉพาะตอน MANUAL ON
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

// ✅ เพิ่ม Event Listener นี้เข้ามา
window.electronAPI.onRobotPose((poseData) => {
  if (poseData.position && poseData.orientation) {
    // 1. อัปเดต State ส่วนกลาง
    updateRobotPose(poseData.position, poseData.orientation);
    // 2. สั่งให้ Canvas วาดใหม่เพื่อแสดงตำแหน่งล่าสุด
    renderDashboardMap();
  }
});

window.electronAPI.onPlannedPath((pathData) => {
  setPlannedPath(pathData);
});

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
    const whepPort = document.getElementById('profile-whep-port').value;

    // แก้ไขเงื่อนไขการตรวจสอบ
    if (!address || !rosPort || !whepPort) {
        statusEl.textContent = '❌ Please fill in all address and port fields.';
        statusEl.style.color = 'red';
        return;
    }

    const rosIp = address;
    const whepUrl = `http://${address}:${whepPort}/live/whep`;

    console.log(`🔌 Connecting to ROSBridge at ${rosIp}:${rosPort}`);
    window.electronAPI.connectROSBridge(rosIp);

    // ... (RTC Player connection) ...
     const videoElement = document.getElementById('stream');
    const webrtcStatusElement = document.getElementById('rtc_status');
    
    if (rtcPlayer) rtcPlayer.disconnect();
    rtcPlayer = new WebRTCPlayer(whepUrl, videoElement, webrtcStatusElement);
    rtcPlayer.connect();

    statusEl.textContent = `🚀 Attempting to connect using profile: ${selectedProfileName}`;
    statusEl.style.color = 'green';
}


