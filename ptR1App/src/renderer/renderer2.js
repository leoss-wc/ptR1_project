// Capture canvas 
const canvas = document.getElementById('capture-canvas');
const ctx = canvas.getContext('2d');
const startBtn = document.getElementById('start-record');
const stopBtn = document.getElementById('stop-record')
// Video elements
const videoPlayer = document.getElementById("video-player"); 
const videoGallery = document.getElementById("video-gallery"); 

// Mode toggle elements
const keyboardToggle = document.getElementById("keyboard-toggle");
const modeLabel = document.getElementById("mode-label");


// imu and magnetometer
const toggle_imu = document.getElementById('toggle-control-imu');
const toggle_mag = document.getElementById('toggle-control-mag');

// Connect status elements
const connectionEl = document.getElementById('connection-status');

let mediaRecorder = null;
let recordedChunks = [];
let recordingInterval = null;
let isRecording = false;
let currentVideoFolder = null; // เก็บ path ปัจจุบันไว้


// Manual mode toggle
keyboardToggle.addEventListener('change', (event) => {
  const isOn = event.target.checked;
  modeLabel.textContent = isOn ? 'MANUAL ON' : 'MANUAL OFF';

  console.log(isOn ? '🛠 Switched to MANUAL ON' : '🛑 Switched to MANUAL OFF');
  window.electronAPI.setManualMode(isOn);
});

async function loadVideos(pathOverride = null) {
  const videoGallery = document.getElementById("video-gallery");
  const videoPlayer = document.getElementById("video-player");

  const videos = await window.electronAPI.loadVideosFromFolder(pathOverride);
  if (pathOverride) currentVideoFolder = pathOverride;

  videoGallery.innerHTML = '';

  for (const { relativePath } of videos) {
    const videoSrc = await window.electronAPI.getVideoFileURL(relativePath);

    const thumb = document.createElement("video");
    thumb.src = videoSrc;
    thumb.className = "video-thumb";
    thumb.muted = true;
    thumb.loop = true;

    thumb.addEventListener("click", () => {
      const source = videoPlayer.querySelector("source");
      source.src = videoSrc;
      videoPlayer.load();
      console.log('🎥 Playing from:', videoSrc);

      videoPlayer.onloadeddata = () => {
        videoPlayer.play().catch((err) => {
          console.warn("🎥 Video play interrupted:", err.message);
        });
      };
    });

    videoGallery.appendChild(thumb);
  }
}

// 📂 ปุ่มเปลี่ยนโฟลเดอร์
document.getElementById("select-folder-btn").addEventListener("click", async () => {
  const folderPath = await window.electronAPI.selectFolder();
  if (folderPath) {
    await loadVideos(folderPath); // ✅ ใช้ path ใหม่
  }
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

// IMU and Magnetometer toggle
toggle_imu.addEventListener('change', () => {
  const variableId = 0x09; // use_imu
  const value = toggle_imu.checked ? 1 : 0;
  window.robotControl.sendCommand_vairable(variableId, value);
});

toggle_mag.addEventListener('change', () => {
  const variableId = 0x0A; // use_imu
  const value = toggle_mag.checked ? 1 : 0;
  window.robotControl.sendCommand_vairable(variableId, value);
});

// เช็คสถานะการเชื่อมต่อ
window.electronAPI.onConnectionStatus((status) => {
  let text = ' Unknown';
  if (status === 'connected') text = ' Connected';
  else if (status === 'disconnected') text = ' Disconnected';
  else if (status === 'error') text = ' Error';

  if (connectionEl) {
    connectionEl.textContent = text;
    connectionEl.className = status;
  }
});

document.getElementById('save-map-btn').addEventListener('click', () => {
  const name = document.getElementById('map-name-input').value.trim();
  if (!name) {
    alert('❗ Please enter a map name');
    return;
  }

  window.electronAPI.saveMap(name);
  document.getElementById('live-control-box').classList.add('hidden');
});


window.electronAPI.onMapSaveResult((res) => {
  if (res.success) {
    alert(`✅ Map "${res.name}" saved successfully!`);
  } else {
    alert(`❌ Failed to save map "${res.name}":\n${res.message}`);
  }
});

document.getElementById('start-patrol-btn').addEventListener('click', () => {
  if (goalPoint) {
    // กรณีจุดเดียว
    console.log("📤 ส่ง goalPoint:", goalPoint);
    window.electronAPI.sendSingleGoal(goalPoint);
  } else if (patrolPath.length > 0) {
    // กรณีหลายจุด
    console.log("📤 ส่ง path:", patrolPath);
    window.electronAPI.sendPatrolPath(patrolPath);
  } else {
    alert("❌ ยังไม่ได้ตั้งจุดหมายหรือ path");
  }
});


// ปุ่ม Stop Patrol → ส่ง stop command
document.getElementById('stop-patrol-btn').addEventListener('click', () => {
  window.electronAPI.sendStopPatrol();
});

// ฟังผลลัพธ์ feedback สถานะหุ่นยนต์
window.electronAPI.onPatrolStatus((isMoving) => {
  const label = document.getElementById('patrol-status-label');
  if (label) {
    label.textContent = isMoving ? "🚶‍♂️ กำลังเคลื่อนที่" : "⏹ หยุดนิ่ง";
    label.style.color = isMoving ? "lightgreen" : "red";
  }
});

document.getElementById('start-slam-btn').addEventListener('click', () => {
  window.electronAPI.startSLAM();
});

document.getElementById('stop-slam-btn').addEventListener('click', () => {
  window.electronAPI.stopSLAM();
});

window.electronAPI.onSLAMStartResult((result) => {
  const label = document.getElementById('slam-result-label');
  if (label) {
    label.textContent = result.success ? "✅ SLAM Started" : `❌ ${result.message}`;
    label.style.color = result.success ? "lightgreen" : "red";
  }
});

window.electronAPI.onSLAMStopResult((result) => {
  const label = document.getElementById('slam-result-label');
  if (label) {
    label.textContent = result.success ? "🛑 SLAM Stopped" : `❌ ${result.message}`;
    label.style.color = result.success ? "orange" : "red";
  }
});


const liveMapCanvas = document.getElementById('liveMapCanvas');
const ctx_live_map = liveMapCanvas.getContext('2d');

let latestMap = null;
let robotPose = null;

// รับข้อมูลจาก ROSBridge
window.electronAPI.onLiveMap((msg) => {
  latestMap = msg;
  renderLiveMap();
});

window.electronAPI.onRobotPose((pose) => {
  robotPose = pose;
  renderLiveMap();  // redraw
});

function renderLiveMap() {
  if (!latestMap) return;

  const { width, height, resolution, origin, data } = extractMap(latestMap);
  const canvasWidth = liveMapCanvas.width;
  const canvasHeight = liveMapCanvas.height;

  // Convert occupancy grid to ImageData
  const img = ctx_live_map.createImageData(width, height);
  for (let i = 0; i < data.length; i++) {
    const value = data[i];
    const color = value === 100 ? 0 : value === 0 ? 255 : 127;
    img.data[i * 4 + 0] = color;
    img.data[i * 4 + 1] = color;
    img.data[i * 4 + 2] = color;
    img.data[i * 4 + 3] = 255;
  }

  // Clear + draw map
  ctx_live_map.clearRect(0, 0, canvasWidth, canvasHeight);
  ctx_live_map.putImageData(img, 0, 0);

  // Draw robot if pose available
  if (robotPose) {
    const { x, y } = robotPose.position;
    const rx = Math.floor((x - origin.x) / resolution);
    const ry = height - Math.floor((y - origin.y) / resolution);

    ctx_live_map.beginPath();
    ctx_live_map.arc(rx, ry, 6, 0, 2 * Math.PI);
    ctx_live_map.fillStyle = 'red';
    ctx_live_map.fill();
  }
}

function extractMap(msg) {
  return {
    width: msg.info.width,
    height: msg.info.height,
    resolution: msg.info.resolution,
    origin: msg.info.origin.position,
    data: msg.data,
  };
}

let currentPatrolIndex = 0;
let isPatrolling = false;

window.electronAPI.onPatrolStatus((isMoving) => {
  isPatrolling = isMoving;
});

// เมื่อหยุดการลาดตระเวน:
currentPatrolIndex = estimateCurrentIndex(); // ต้องอิงตำแหน่งหุ่นยนต์
isPatrolling = false;

function estimateCurrentIndex() {
  if (!robotPose || patrolPath.length === 0) return 0;

  const rx = robotPose.position.x;
  const ry = robotPose.position.y;

  let minDist = Infinity;
  let minIndex = 0;

  patrolPath.forEach((pt, i) => {
    const dist = Math.hypot(pt.x - rx, pt.y - ry);
    if (dist < minDist) {
      minDist = dist;
      minIndex = i;
    }
  });

  return minIndex + 1; // เริ่มจากจุดถัดไป
}

document.getElementById('resume-patrol-btn').addEventListener('click', () => {
  if (!isPatrolling && patrolPath.length > 0) {
    const resumeIndex = estimateCurrentIndex();
    console.log(`🔄 Resume Patrol from index ${resumeIndex}`);
    window.electronAPI.resumePatrol(patrolPath, resumeIndex);
  }
});



async function loadActiveMapMeta() {
  if (!activeMap?.name) return;

  const result = await window.electronAPI.getMapMeta(activeMap.name);
  if (result.success) {
    activeMap.meta = result.data;  // 👈 เพิ่ม meta เข้าไป
    console.log("✅ Loaded map meta:", result.data);
  } else {
    console.warn("❌ Failed to load map meta:", result.message);
  }
}

// 🔧 ค่า global
let zoom_home = 1.0;
let offset_home = { x: 0, y: 0 };

// 📌 Core render call
function renderHomeCanvas() {
  const canvas = document.getElementById('homeMapCanvas');
  if (!canvas || !activeMap?.base64 || !activeMap?.meta || !robotPose) return;

  const ctx = canvas.getContext('2d');
  const width = canvas.width = canvas.clientWidth;
  const height = canvas.height = canvas.clientHeight;

  const mapImg = new Image();
  mapImg.onload = () => {
    ctx.clearRect(0, 0, width, height);

    drawMap(ctx, canvas, mapImg);
    drawPatrolPath(ctx, canvas, mapImg);
    drawRobot(ctx, canvas, mapImg);
  };
  mapImg.src = activeMap.base64;
}
function drawMap(ctx, canvas, mapImg) {
  const imgWidth = mapImg.width * zoom_home;
  const imgHeight = mapImg.height * zoom_home;

  const offsetX = canvas.width / 2 - imgWidth / 2 + offset_home.x;
  const offsetY = canvas.height / 2 - imgHeight / 2 + offset_home.y;

  ctx.save();
  ctx.translate(offsetX, offsetY);
  ctx.scale(zoom_home, zoom_home);
  ctx.drawImage(mapImg, 0, 0);
  ctx.restore();
}
function drawRobot(ctx, canvas, mapImg) {
  if (!robotPose || !activeMap?.meta) return;

  const { resolution, origin } = activeMap.meta;
  const originX = origin[0], originY = origin[1];
  const imgH = mapImg.height;

  const wx = robotPose.position.x;
  const wy = robotPose.position.y;

  // 🔁 แปลง world → pixel
  const px = (wx - originX) / resolution;
  const py = imgH - (wy - originY) / resolution;

  const yaw = getYawFromQuaternion(robotPose.orientation);

  ctx.save();
  ctx.translate(offset_home.x, offset_home.y);               // pan
  ctx.translate(canvas.width / 2, canvas.height / 2);        // center
  ctx.scale(zoom_home, zoom_home);                           // zoom
  ctx.translate(px, py);                                     // world-to-pixel
  ctx.rotate(yaw);                                           // direction

  // 🔴 draw arrow
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(10, 0);
  ctx.strokeStyle = 'red';
  ctx.lineWidth = 2;
  ctx.stroke();

  ctx.beginPath();
  ctx.arc(0, 0, 5, 0, Math.PI * 2);
  ctx.fillStyle = 'red';
  ctx.fill();

  ctx.restore();
}
function drawRobot(ctx, canvas, mapImg) {
  if (!robotPose || !activeMap?.meta) return;

  const { resolution, origin } = activeMap.meta;
  const originX = origin[0], originY = origin[1];
  const imgH = mapImg.height;

  const wx = robotPose.position.x;
  const wy = robotPose.position.y;

  const px = (wx - originX) / resolution;
  const py = imgH - (wy - originY) / resolution;

  const yaw = getYawFromQuaternion(robotPose.orientation);

  const screenX = px * zoom_home + offset_home.x + canvas.width / 2;
  const screenY = py * zoom_home + offset_home.y + canvas.height / 2;

  ctx.save();
  ctx.translate(offset_home.x, offset_home.y);
  ctx.translate(canvas.width / 2, canvas.height / 2);
  ctx.scale(zoom_home, zoom_home);
  ctx.translate(px, py);
  ctx.rotate(yaw);
  // ... draw arrow ...
  ctx.restore();

}
function getYawFromQuaternion(q) {
  return Math.atan2(
    2 * (q.w * q.z + q.x * q.y),
    1 - 2 * (q.y * q.y + q.z * q.z)
  );
}

const homeCanvas = document.getElementById('homeMapCanvas');

let isDraggingHome = false;
let dragStartHome = { x: 0, y: 0 };

homeCanvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  const zoomFactor = 1.1;
  const oldZoom = zoom_home;

  zoom_home = e.deltaY < 0
    ? Math.min(zoom_home * zoomFactor, 10)
    : Math.max(zoom_home / zoomFactor, 0.2);

  const rect = homeCanvas.getBoundingClientRect();
  const mouseX = e.clientX - rect.left;
  const mouseY = e.clientY - rect.top;

  const dx = (mouseX - offset_home.x - homeCanvas.width / 2) * (zoom_home / oldZoom - 1);
  const dy = (mouseY - offset_home.y - homeCanvas.height / 2) * (zoom_home / oldZoom - 1);

  offset_home.x -= dx;
  offset_home.y -= dy;

  renderHomeCanvas();
});

homeCanvas.addEventListener('mousedown', (e) => {
  isDraggingHome = true;
  dragStartHome = { x: e.clientX, y: e.clientY };
});

homeCanvas.addEventListener('mouseup', () => isDraggingHome = false);
homeCanvas.addEventListener('mouseleave', () => isDraggingHome = false);
homeCanvas.addEventListener('mousemove', (e) => {
  if (!isDraggingHome) return;
  offset_home.x += e.clientX - dragStartHome.x;
  offset_home.y += e.clientY - dragStartHome.y;
  dragStartHome = { x: e.clientX, y: e.clientY };
  renderHomeCanvas();
});


window.electronAPI.onRobotPose((data) => {
  robotPose = data;
  renderHomeCanvas();
});

// อ้างอิงถึง element ต่างๆ ใน DOM
const openDialogBtn = document.getElementById('addRobotBtn');

const robotDialog = document.getElementById('robotDialog');
const cancelBtn = document.getElementById('cancelBtn');
const confirmBtn = document.getElementById('confirmBtn');
const robotNameInput = document.getElementById('robotName');
const robotIPInput = document.getElementById('robotIP');
const whepPortInput = document.getElementById('whepPort');
const rosPortInput = document.getElementById('rosPort');

let robotList = [];

async function init() {
  robotList = await window.robotAPI.loadRobots();
  updateRobotDropdown();
}

function updateRobotDropdown() {
  const dropdown = document.getElementById('robotDropdown');
  dropdown.innerHTML = '';
  robotList.forEach((robot, index) => {
    const opt = document.createElement('option');
    opt.value = index;
    opt.textContent = `${robot.name} (${robot.ip})`;
    dropdown.appendChild(opt);
  });
}

// --- Event Listener สำหรับปุ่มเปิด Dialog ---
openDialogBtn.addEventListener('click', () => {
  console.log('Dialog opened.');
  robotDialog.showModal();
});

// --- Event Listener สำหรับปุ่ม Cancel ใน Dialog ---
cancelBtn.addEventListener('click', () => {
  console.log('Dialog cancelled.');
  clearForm();
  robotDialog.close();
});

// --- Event Listener สำหรับปุ่ม Save ใน Dialog ---
confirmBtn.addEventListener('click',async (event) => {
  event.preventDefault();
  if (!robotNameInput.value || !robotIPInput.value || !whepPortInput.value || !rosPortInput.value) {
    alert('Please fill in all fields.');
    return; 
  }

  // สร้าง object เพื่อเก็บข้อมูลหุ่นยนต์
  const newRobot = {
    name: robotNameInput.value,
    ipAddress: robotIPInput.value,
    whepPort: parseInt(whepPortInput.value), // แปลงเป็นตัวเลข
    rosPort: parseInt(rosPortInput.value)    // แปลงเป็นตัวเลข
  };
  console.log('New Robot Data:', newRobot);
  robotList.push(newRobot); 
  console.log('Updated Robot List:', robotList);

  //await window.robotAPI.saveRobots(robotList);
  updateRobotDropdown();
  alert('✅ Robot saved');
  robotDialog.close();
  clearForm();
});

// --- Function สำหรับเคลียร์ค่าในฟอร์ม ---
function clearForm() {
  robotNameInput.value = '';
  robotIPInput.value = '';
  whepPortInput.value = '';
  rosPortInput.value = '';
}

// --- Optional: ตรวจจับเหตุการณ์ 'close' ของ dialog (เมื่อถูกปิดไม่ว่าจะด้วยวิธีใด) ---
robotDialog.addEventListener('close', () => {
  console.log('Dialog was close.');
  clearForm();
});

document.getElementById('robotDropdown').addEventListener('change', () => {
  const selected = robotList[document.getElementById('robotDropdown').value];
  console.log('Selected robot:', selected);
  // ใช้ selected.ip, selected.whepPort, selected.rosPort ได้เลย
});

