// modules/profileManager.js
import { WebRTCPlayer } from './webrtcPlayer.js';
import { FrameProcessor } from './FrameProcessor.js';
import { OverlayCanvas } from './OverlayCanvas.js';

let allRobotProfiles = [];
let selectedProfileName = null;
let rtcPlayer = null;
let frameProcessor = null;
let yoloOverlay = null;

export async function initProfileManager() {
  document.getElementById('robot-profile-select').addEventListener('change', handleProfileSelection);
  document.getElementById('add-profile-btn').addEventListener('click', addNewProfile);
  document.getElementById('save-profile-btn').addEventListener('click', saveProfile);
  document.getElementById('delete-profile-btn').addEventListener('click', deleteProfile);
  document.getElementById('connect-all-btn').addEventListener('click', connectUsingCurrentProfile);
  document.getElementById('connect-video-btn').addEventListener('click', connectVideoPlayer);
  document.getElementById('connectButton').addEventListener('click', connectUsingCurrentProfile);

  await loadAndDisplayProfiles();
  console.log('Profile Manager: Initialized profile manager and loaded profiles.');
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
        statusEl.textContent = '✅ Profile saved successfully!';
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

function connectVideoPlayer() {
    const address = document.getElementById('profile-address').value;
    const whepPort = document.getElementById('profile-whep-port').value;
    if (!address || !whepPort) return;

    const whepUrl = `http://${address}:${whepPort}/mystream/whep`;
    const videoElement = document.getElementById('stream');
    const webrtcStatusElement = document.getElementById('rtc_status');
    
    if (rtcPlayer) rtcPlayer.disconnect();
    rtcPlayer = new WebRTCPlayer(whepUrl, videoElement, webrtcStatusElement);
    rtcPlayer.connect();

    if (yoloOverlay) yoloOverlay.clear();
    yoloOverlay = new OverlayCanvas('yolo-overlay', videoElement);
    
    if (frameProcessor) frameProcessor.stop();
    // สร้าง Instance ใหม่ของ FrameProcessor
    frameProcessor = new FrameProcessor(videoElement, (detections) => {
        if (yoloOverlay) yoloOverlay.drawDetections(detections);
        // (Optional) ถ้า FrameProcessor ตัวใหม่มีฟังก์ชัน sendFrame()
        // เราสามารถสั่ง sendFrame() ต่อจากตรงนี้ได้ ถ้าอยากคุม Flow เองแบบ Manual สุดๆ
        // แต่ใน Class ที่ผมให้ไป มันจัดการเรื่องนี้ให้ใน onmessage แล้ว ดังนั้นไม่ต้องทำอะไรเพิ่มครับ
    });
    frameProcessor.start(); 
    videoElement.onloadedmetadata = () => {
        if(yoloOverlay) yoloOverlay.resize();
    };
}