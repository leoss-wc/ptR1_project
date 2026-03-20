// modules/profileManager.js
import { WebRTCPlayer } from './webrtcPlayer.js';

let allRobotProfiles = [];
let selectedProfileName = null;

let rtcPlayer = null;

export async function initProfileManager() {
  document.getElementById('robot-profile-select').addEventListener('change', handleProfileSelection);
  document.getElementById('add-profile-btn').addEventListener('click', addNewProfile);
  document.getElementById('save-profile-btn').addEventListener('click', saveProfile);
  document.getElementById('delete-profile-btn').addEventListener('click', deleteProfile);
  document.getElementById('connect-all-btn').addEventListener('click', connectUsingCurrentProfile);
  //document.getElementById('connect-video-btn').addEventListener('click', startRobotStream);
  document.getElementById('connectButton').addEventListener('click', connectUsingCurrentProfile);
  const startBtn = document.getElementById('start-stream-btn');
  const stopBtn = document.getElementById('stop-stream-btn');
  // ปุ่ม Start
  startBtn.addEventListener('click', async () => {
      startBtn.disabled = true;
      startBtn.innerText = "Starting...";
      
      const success = await startRobotStream(); // เรียกฟังก์ชันที่เราเขียนใหม่
      
      if (success) {
          startBtn.innerText = "Streaming (Active)";
          stopBtn.disabled = false;
      } else {
          startBtn.disabled = false;
          startBtn.innerText = "Start Stream";
          alert("Connection Failed");
      }
  });

  // ปุ่ม Stop
  stopBtn.addEventListener('click', async () => {
      stopBtn.disabled = true;
      stopBtn.innerText = "Stopping...";
      
      await stopRobotStream(); // เรียกฟังก์ชัน cleanup ใหม่
      
      startBtn.disabled = false;
      startBtn.innerText = "Start Stream";
      stopBtn.innerText = "Stop Stream";
  });

  await loadAndDisplayProfiles();
  console.log('Profile Manager: Initialized profile manager and loaded profiles.');
}

export function initDatasetCollector() {
    document.addEventListener('keydown', (event) => {
        if (event.code === 'Space' && event.target.tagName !== 'INPUT') {
            event.preventDefault();
            captureSingleFrame();
        }
    });

    console.log("Dataset Collector Ready: Press SPACEBAR to capture a frame.");
}

function captureSingleFrame() {
    const videoElement = document.getElementById('stream');
    const canvas = document.getElementById('capture-canvas');
    if (!videoElement || videoElement.paused || videoElement.ended) {
        console.warn("Video is not playing. Cannot capture.");
        return;
    }

    const context = canvas.getContext('2d');
    
    // ตั้งขนาด Canvas ให้พอดีกับวิดีโอ
    canvas.width = videoElement.videoWidth || 640;
    canvas.height = videoElement.videoHeight || 480;

    // วาดภาพปัจจุบันลง Canvas
    context.drawImage(videoElement, 0, 0, canvas.width, canvas.height);
    
    // แปลงภาพเป็น Base64 (JPEG คุณภาพ 90%)
    const base64Image = canvas.toDataURL('image/jpeg', 0.9);

    // ส่งภาพไปให้ main.js
    if (window.electronAPI && window.electronAPI.saveDatasetImage) {
        window.electronAPI.saveDatasetImage(base64Image);
        showFlashEffect();
    }
}
function showFlashEffect() {
    // หา div ที่ครอบวิดีโออยู่
    const videoContainer = document.querySelector('.video-stream');
    if (!videoContainer) return;

    // สร้างแผ่นสีขาวทับหน้าจอ
    const flash = document.createElement('div');
    flash.style.position = 'absolute';
    flash.style.top = '0';
    flash.style.left = '0';
    flash.style.width = '100%';
    flash.style.height = '100%';
    flash.style.backgroundColor = 'white';
    flash.style.opacity = '0.5'; // ความสว่างของแฟลช
    flash.style.pointerEvents = 'none'; // ให้คลิกทะลุได้
    flash.style.transition = 'opacity 0.15s ease-out'; // ความเร็วในการเฟดออก
    flash.style.zIndex = '999';
    
    videoContainer.appendChild(flash);
    
    // สั่งให้แฟลชจางหายไปอย่างรวดเร็ว (เหมือนกดชัตเตอร์กล้อง)
    setTimeout(() => {
        flash.style.opacity = '0';
        setTimeout(() => flash.remove(), 150);
    }, 50);
}
export async function startRobotStream() {
    const address = document.getElementById('profile-address').value;
    const whepPort = document.getElementById('profile-whep-port').value; 
    
    if (!address || !whepPort) {
        alert("Please select a profile.");
        return false;
    }

    console.log("🚀 Starting Stream Sequence...");

    // 1. สั่ง Backend ให้เริ่ม
    const backendSuccess = await window.electronAPI.startFFmpegStream();
    if (!backendSuccess) {
        console.error("❌ Failed to start Backend.");
        return false;
    }
    // 2.ใช้ระบบรอแบบ วนเช็คทุก 1 วินาที สูงสุด 15 ครั้ง
    console.log("⏳ Polling for stream availability...");
    const isReady = await waitForStreamReady(address, whepPort, 10);

    if (isReady) {
        console.log("Stream is online! Connecting Player...");
        connectPlayer(address, whepPort);
        return true;
    } else {
        console.error("❌ Stream timeout. Backend started but no video signal.");
        alert("Stream started but timed out (No Video). Check Camera/Network.");
        // อาจจะสั่ง stop เพื่อเคลียร์ process
        stopRobotStream();
        return false;
    }
}

async function waitForStreamReady(address, port, maxRetries = 10) {
    const checkUrl = `http://${address}:${port}/mystream/whep`; // URL เดียวกับที่ Player ใช้

    for (let i = 0; i < maxRetries; i++) {
        try {            
            console.log(`Checking stream attempt ${i + 1}/${maxRetries}...`);
            // 1. ลองยิง Request ไปเช็ค (ใช้ timeout สั้นๆ 1 วินาทีเผื่อ Server ค้าง)
            const controller = new AbortController();
            const timeoutId = setTimeout(() => controller.abort(), 1000);
            // ใช้ method 'HEAD' เพื่อเช็คแค่ว่า Server อยู่ไหม
            const response = await fetch(checkUrl, { 
                method: 'HEAD', 
                signal: controller.signal 
            }).catch(() => null);
            clearTimeout(timeoutId);
            // 2. ถ้า Server ตอบกลับมา (ไม่ว่าจะ 200, 404 หรือ 405) แปลว่า Port เปิดแล้ว!
            if (response) {
                console.log("Server is responding! Ready to connect.");
                
                // รอแถมอีกนิดนึง (1 วิ) ให้ FFmpeg ส่งข้อมูลเฟรมแรกเข้าทัน
                await new Promise(r => setTimeout(r, 1000)); 
                
                return true; // 👈 ออกจาก Loop ทันที ไม่ต้องรอครบ 10 วิ
            }
            
        } catch (e) {
            console.log("Waiting...");
        }
        await new Promise(r => setTimeout(r, 1000)); // รอทีละ 1 วินาที
    }
    return true; // ถึงจะครบ 10 ครั้งก็ถือว่าใช้ได้
}

export async function stopRobotStream() {
    console.log("Stopping Stream Sequence...");
    disconnectPlayer();
    // 2. สั่ง Backend ให้ปิด FFmpeg
    await window.electronAPI.stopFFmpegStream();
    console.log("Stream stopped and cleaned up.");
}

export function connectPlayer(address, whepPort) {
    console.log("Initializing Player ...");
    if (!address || !whepPort) {
        console.error("❌ Missing Address or Port");
        return;
    }
    disconnectPlayer();
    const whepUrl = `http://${address}:${whepPort}/mystream/whep`;
    const videoElement = document.getElementById('stream');
    const statusElement = document.getElementById('rtc_status');
    const overlayEl = document.getElementById('disconnect-overlay');
    // Setup WebRTC Player
    rtcPlayer = new WebRTCPlayer(whepUrl, videoElement, statusElement, overlayEl);
    rtcPlayer.connect();
    // Handler เมื่อวิดีโอเริ่มเล่น
    const handlePlay = () => {
        console.log("▶ Video playing logic triggered.");
    };
    videoElement.onplaying = handlePlay;
    // ถ้าวิดีโอมันเล่นอยู่แล้ว (Already Playing) ให้เรียกเลยไม่ต้องรอ Event
    if (!videoElement.paused && !videoElement.ended && videoElement.readyState > 2) {
        handlePlay();
    }
}


export function disconnectPlayer() {
    if (window.overlay) {
        window.overlay.clear();
    }
    // หยุด Player
    if (rtcPlayer) {
        rtcPlayer.disconnect();
        rtcPlayer = null;
    }
    
    // ล้าง Event
    const videoElement = document.getElementById('stream');
    if (videoElement) {
        videoElement.onplaying = null;
    }
}

async function loadAndDisplayProfiles() {
    const profileSelect = document.getElementById('robot-profile-select');
    profileSelect.innerHTML = '<option value="">-- Select a Profile --</option>';
    allRobotProfiles = await window.electronAPI.loadRobots();
    
    if (allRobotProfiles && allRobotProfiles.length > 0) {
        allRobotProfiles.forEach(profile => {
            const option = document.createElement('option');
            option.value = profile.name;
            option.textContent = profile.name;
            profileSelect.appendChild(option);
        });
    }
    const lastSelected = localStorage.getItem('lastSelectedProfile');
    if (lastSelected && allRobotProfiles.some(p => p.name === lastSelected)) {
        profileSelect.value = lastSelected;
        handleProfileSelection();
    } else {
        document.getElementById('profile-form-section').style.display = 'none';
    }
}

function handleProfileSelection() {
    const profileSelect = document.getElementById('robot-profile-select');
    selectedProfileName = profileSelect.value;
    const formSection = document.getElementById('profile-form-section');
    
    if (!selectedProfileName) {
        formSection.style.display = 'none';
        updateHomePanel(null);
        return;
    }
    const profileData = allRobotProfiles.find(p => p.name === selectedProfileName);
    if (profileData) {
        document.getElementById('profile-name').value = profileData.name || '';
        document.getElementById('profile-address').value = profileData.address || '';
        document.getElementById('profile-ros-port').value = profileData.rosPort || '9090';
        document.getElementById('profile-whep-port').value = profileData.whepPort || '8889';
        document.getElementById('profile-name').disabled = true; 
        document.getElementById('form-title').textContent = `Editing: ${profileData.name}`;
        formSection.style.display = 'block';
        localStorage.setItem('lastSelectedProfile', selectedProfileName);
        updateHomePanel(profileData);
    }
}

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
    document.getElementById('profile-name').value = '';
    document.getElementById('profile-address').value = ''; 
    document.getElementById('profile-ros-port').value = '9090';
    document.getElementById('profile-whep-port').value = '8889';
    document.getElementById('profile-name').disabled = false; 
    document.getElementById('profile-name').focus();
    document.getElementById('form-title').textContent = '➕ Add New Profile';
    selectedProfileName = null; 
    document.getElementById('robot-profile-select').value = '';
    document.getElementById('profile-form-section').style.display = 'block';
    updateHomePanel(null);
}

async function saveProfile() {
    const statusEl = document.getElementById('settings-status');
    const newName = document.getElementById('profile-name').value.trim();
    if (!newName) return;

    const updatedProfileData = {
        name: newName,
        address: document.getElementById('profile-address').value.trim(),
        rosPort: parseInt(document.getElementById('profile-ros-port').value, 10),
        whepPort: parseInt(document.getElementById('profile-whep-port').value, 10),
    };

    if (selectedProfileName) {
        const index = allRobotProfiles.findIndex(p => p.name === selectedProfileName);
        if (index > -1) allRobotProfiles[index] = updatedProfileData;
    } else {
        if (allRobotProfiles.some(p => p.name === newName)) {
            statusEl.textContent = '❌ Profile name already exists.';
            return;
        }
        allRobotProfiles.push(updatedProfileData);
    }
    
    const result = await window.electronAPI.saveRobots(allRobotProfiles);
    if (result) {
        statusEl.textContent = 'Profile saved successfully!';
        statusEl.style.color = 'green';
        await loadAndDisplayProfiles();
        document.getElementById('robot-profile-select').value = newName;
        handleProfileSelection();
    }
}

async function deleteProfile() {
    if (!selectedProfileName) return;
    if (confirm(`Delete "${selectedProfileName}"?`)) {
        allRobotProfiles = allRobotProfiles.filter(p => p.name !== selectedProfileName);
        await window.electronAPI.saveRobots(allRobotProfiles);
        await loadAndDisplayProfiles();
        document.getElementById('settings-status').textContent = `🗑️ Profile deleted.`;
    }
}

export function connectUsingCurrentProfile() {
    const statusEl = document.getElementById('settings-status');
    const address = document.getElementById('profile-address').value;
    const rosPort = document.getElementById('profile-ros-port').value;
    if (!address || !rosPort) return;

    console.log(`🔌 Connecting to ROSBridge at ${address}:${rosPort}`);
    window.electronAPI.connectROSBridge(address, rosPort);
    statusEl.textContent = `Connecting to ${selectedProfileName}...`;
}
