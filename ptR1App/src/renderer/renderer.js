

console.log("Renderer process started**********");

// Capture canvas 
const canvas = document.getElementById('capture-canvas');
const ctx = canvas.getContext('2d');
const startBtn = document.getElementById('start-record');
const stopBtn = document.getElementById('stop-record')

// Removed unused video elements
const modeLabel = document.getElementById("mode-label");

// ผูก event relay button
document.getElementById("relayButton1").addEventListener("click", () => toggleRelay("relayButton1"));
document.getElementById("relayButton2").addEventListener("click", () => toggleRelay("relayButton2"));

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

const RECORD_DURATION_MS = 10 * 60 * 1000; // 10 นาที
//const RECORD_DURATION_MS = 10 * 1000; // 10 วินาที


function startNewRecordingCycle() {
  const stream = canvas.captureStream(15); // 15 FPS
  recordedChunks = [];

  mediaRecorder = new MediaRecorder(stream, { mimeType: 'video/webm' });

  mediaRecorder.ondataavailable = (e) => {
    if (e.data.size > 0) recordedChunks.push(e.data);
  };

  mediaRecorder.onstop = () => {
    const blob = new Blob(recordedChunks, { type: 'video/webm' });

    if (blob.size === 0) {
      console.warn("⚠️ Skipped empty recording (0 byte)");
      if (isRecording) startNewRecordingCycle();
      return;
    }
  
    // สร้าง Object URL เพื่อให้ดาวน์โหลดทันที (optional)
    // const url = URL.createObjectURL(blob);
    // const a = document.createElement('a');
    // a.href = url;
    // a.download = `stream_${new Date().toISOString().replace(/[:.]/g, '-')}.webm`;
    // a.click();
  
    // แยกวันและเวลา
    const now = new Date();
    const dateStr = now.toISOString().split('T')[0]; // yyyy-mm-dd
    const timeStr = now.toTimeString().slice(0, 5).replace(':', '-'); // hh-mm
  
    // ส่ง blob ไป main process
    blob.arrayBuffer().then((arrayBuf) => {
      window.electronAPI.saveVideo({
        buffer: arrayBuf,
        date: dateStr,
        filename: `record-${timeStr}.webm`
      });
  
      // เริ่มรอบถัดไปถ้ายังไม่หยุด
      if (isRecording) {
        startNewRecordingCycle();
      }
    });
  };
  

  mediaRecorder.onstart = () => {
    recordingInterval = setTimeout(() => {
      if (mediaRecorder && mediaRecorder.state === 'recording') {
        mediaRecorder.stop();
      }
    }, RECORD_DURATION_MS);
  };
  
  mediaRecorder.start();

}
// ▶️ เริ่มบันทึกวนลูป
startBtn.addEventListener('click', () => {
  if (isRecording) return;
  isRecording = true;
  startNewRecordingCycle();
  startBtn.disabled = true;
  stopBtn.disabled = false;
});

stopBtn.addEventListener('click', () => {
  if (!isRecording) return;
  isRecording = false;

  clearTimeout(recordingInterval);

  if (mediaRecorder && mediaRecorder.state === 'recording') {
    mediaRecorder.stop(); // stop ล่าสุด → onstop จะไม่เรียกถัดไปแล้ว
  }

  startBtn.disabled = false;
  stopBtn.disabled = true;
});

//relay button states
const relayStates = {
  relay1: false,
  relay2: false
};

// Map ปุ่ม HTML ID → ชื่อ relay ใน ROS
const relayIdMap = {
  relayButton1: 'relay1',
  relayButton2: 'relay2'
};

// relay button update
function updateButton(buttonId) {
  const relayId = relayIdMap[buttonId];
  const btn = document.getElementById(buttonId);
  btn.textContent = `${relayId.toUpperCase()}: ${relayStates[relayId] ? "ON" : "OFF"}`;

  if (relayStates[relayId]) {
    btn.classList.remove("off");
    btn.classList.add("on");
  } else {
    btn.classList.remove("on");
    btn.classList.add("off");
  }
}
// relay toggle
function toggleRelay(buttonId) {
  const relayId = relayIdMap[buttonId];
  relayStates[relayId] = !relayStates[relayId];
  updateButton(buttonId);
  window.electronAPI.sendRelayCommand(relayId, relayStates[relayId] ? "on" : "off");
}

// Manual mode toggle
keyboardToggle.addEventListener('change', (event) => {
  const isOn = event.target.checked;
  modeLabel.textContent = isOn ? 'MANUAL ON' : 'MANUAL OFF';

  console.log(isOn ? '🛠 Switched to MANUAL ON' : '🛑 Switched to MANUAL OFF');
  window.electronAPI.setManualMode(isOn);
});

// ...existing code...

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
 
// 📦 Map Interface
const canvas_map = document.getElementById('staticMapCanvas');
let mapImage = null;
let zoom = 1.0;
let offsetX = 0;
let offsetY = 0;
let isDragging = false;
let dragStartX = 0;
let dragStartY = 0;
const patrolPath = [];
let mode = 'none'; 
let goalPoint = null; // { x, y } หรือ null
let current_map_select = { name: null, base64: null };
let activeMap = { name: null, base64: null };


document.getElementById('sync-maps-btn').addEventListener('click', () => {
  window.electronAPI.syncMaps();
});


window.electronAPI.onSyncComplete((mapList) => {
  const gallery = document.getElementById('map-gallery');
  gallery.innerHTML = '';

  mapList.forEach(({ name, base64 }, index) => {
    const img = document.createElement('img');
    img.src = base64;
    img.alt = name;
    img.title = name;
    img.className = 'map-thumb';

    img.addEventListener('click', () => {
      showOnCanvas(name, base64);    });

    gallery.appendChild(img);
  });
  loadLocalMapsToGallery();

});

async function loadLocalMapsToGallery() {
  const maps = await window.electronAPI.getLocalMaps();
  if (maps.length === 0) return;

  const gallery = document.getElementById('map-gallery');
  gallery.innerHTML = '';

  maps.forEach(({ name, base64 }, index) => {
    const img = document.createElement('img');
    img.src = base64;
    img.alt = name;
    img.title = name;
    img.className = 'map-thumb';
    img.style.cursor = 'pointer';

    img.addEventListener('click', () => {
      showOnCanvas(name, base64);
    });
    gallery.appendChild(img);
  });
}

document.getElementById('open-map-folder-btn').addEventListener('click', async () => {
  const defaultMapPath = await window.electronAPI.getUserDataPath('maps');
  const folderPath = await window.electronAPI.selectFolder(defaultMapPath);
  if (folderPath) {
    console.log('📂 Selected folder:', folderPath);
    // TODO: อาจโหลดไฟล์ในโฟลเดอร์นี้มาดู หรือ set เป็น workspace ต่อไป
  }
});

function showOnCanvas(name, base64) {
  const ctx = canvas_map.getContext('2d');

  mapImage = new Image();
  mapImage.onload = () => {
    zoom = 1.0;
    offsetX = 0;
    offsetY = 0;
    renderCanvas();
  };
  mapImage.src = base64;
  current_map_select = { name, base64 };
}

function renderCanvas() {
  if (!mapImage) {
    console.log("return because mapImage is null");
    return;
  }
  canvas_map.width = canvas_map.clientWidth;
  canvas_map.height = canvas_map.clientHeight;;

  const ctx = canvas_map.getContext('2d');
  ctx.clearRect(0, 0, canvas_map.width, canvas_map.height);

  const imgWidth = mapImage.width * zoom;
  const imgHeight = mapImage.height * zoom;
  const centerX = canvas_map.width / 2;
  const centerY = canvas_map.height / 2;

  ctx.drawImage(
    mapImage,
    centerX - imgWidth / 2 + offsetX,
    centerY - imgHeight / 2 + offsetY,
    imgWidth,
    imgHeight
  );

  if (current_map_select.name) {
  ctx.font = '16px sans-serif';
  ctx.fillStyle = 'white';
  ctx.textBaseline = 'top';
  ctx.fillText(`Map: ${current_map_select.name}`, 10, 10);
}

  // 🟠 วาด path
  if (patrolPath.length > 0) {
    ctx.strokeStyle = 'orange';
    ctx.lineWidth = 2;
    ctx.beginPath();

    patrolPath.forEach((pt, i) => {
      const drawX = canvas_map.width / 2 + pt.x * zoom + offsetX;
      const drawY = canvas_map.height / 2 + pt.y * zoom + offsetY;
      if (i === 0) {
        ctx.moveTo(drawX, drawY);
      } else {
        ctx.lineTo(drawX, drawY);
      }
    });

    // ปิดลูป
    ctx.closePath();
    ctx.stroke();

    // 🔵 วาดจุด
    ctx.fillStyle = 'cyan';
    patrolPath.forEach((pt) => {
      const drawX = canvas_map.width / 2 + pt.x * zoom + offsetX;
      const drawY = canvas_map.height / 2 + pt.y * zoom + offsetY;
      ctx.beginPath();
      ctx.arc(drawX, drawY, 4, 0, Math.PI * 2);
      ctx.fill();
    });
  }
  // 🎯 วาดจุด goal
  if (goalPoint) {
    const drawX = canvas_map.width / 2 + goalPoint.x * zoom + offsetX;
    const drawY = canvas_map.height / 2 + goalPoint.y * zoom + offsetY;

    ctx.fillStyle = 'red';
    ctx.beginPath();
    ctx.arc(drawX, drawY, 6, 0, Math.PI * 2);
    ctx.fill();

    // เพิ่มวงกลมขอบก็ได้
    ctx.strokeStyle = 'white';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(drawX, drawY, 6, 0, Math.PI * 2);
    ctx.stroke();
  }
}
// 📦 Zoom Controls
document.getElementById('zoom-in').addEventListener('click', () => {
   if (!mapImage){
    console.log("return because mapImage is null (zoom in)");
    return;
   }
  zoom = Math.min(zoom * 1.2, 10); // max 10x
  renderCanvas();
});

document.getElementById('zoom-out').addEventListener('click', () => {
  if (!mapImage){
    console.log("return because mapImage is null (zoom out)");
    return;
  }
  zoom = Math.max(zoom / 1.2, 0.1); // min 0.1x
  renderCanvas();
});

document.getElementById('reset-view').addEventListener('click', () => {
  if (!mapImage){
    console.log("return because mapImage is null (reset view)");
    return;
  }
  zoom = 1.0;
  offsetX = 0;
  offsetY = 0;
  renderCanvas();
});

document.getElementById('clear-path-btn').addEventListener('click', () => {
  patrolPath.length = 0;
  cancelMode()
  console.log('Cleared Path and Point Goal');
});

document.getElementById('save-path-btn').addEventListener('click', () => {
  console.log('💾 Saved Path:', patrolPath);
  // TODO: ส่ง path นี้ไปยัง ROS หรือบันทึกลงไฟล์
});

// 🖱 Mouse drag to pan
canvas_map.addEventListener('mousedown', (e) => {
  isDragging = true;
  dragStartX = e.clientX;
  dragStartY = e.clientY;
});

canvas_map.addEventListener('mouseup', () => {
  isDragging = false;
});

canvas_map.addEventListener('mouseleave', () => {
  isDragging = false;
});

canvas_map.addEventListener('mousemove', (e) => {
  if (!isDragging) return;
  const dx = e.clientX - dragStartX;
  const dy = e.clientY - dragStartY;
  dragStartX = e.clientX;
  dragStartY = e.clientY;

  offsetX += dx;
  offsetY += dy;
  renderCanvas();
});

canvas_map.addEventListener('wheel', (e) => {
  if (!mapImage) return;

  e.preventDefault();

  const zoomFactor = 1.1;
  const oldZoom = zoom;

  const rect = canvas_map.getBoundingClientRect();
  const mouseX = e.clientX - rect.left - canvas_map.width / 2;
  const mouseY = e.clientY - rect.top - canvas_map.height / 2;


  if (e.deltaY < 0) {
    zoom = Math.min(zoom * zoomFactor, 10);
  } else {
    zoom = Math.max(zoom / zoomFactor, 0.1);
  }

  const scaleChange = zoom / oldZoom;
   //console.log(`🖱️ mouse at (${mouseX}, ${mouseY}), zoom=${zoom}`);

  // ✅ คำนวณ offset ใหม่ จากจุดเมาส์
  offsetX = mouseX - (mouseX - offsetX) * scaleChange;
  offsetY = mouseY - (mouseY - offsetY) * scaleChange;

  renderCanvas();
});

canvas_map.addEventListener('click', (e) => {
  if (!isDrawMode || isDragging) return;

  const rect = canvas_map.getBoundingClientRect();
  const clickX = e.clientX - rect.left - canvas_map.width / 2;
  const clickY = e.clientY - rect.top - canvas_map.height / 2;

  const x = (clickX - offsetX) / zoom;
  const y = (clickY - offsetY) / zoom;

  patrolPath.push({ x, y });
  renderCanvas();
});

window.addEventListener('resize', () => {
  renderCanvas(); // จะขยาย canvas และวาดใหม่
});

const setGoalBtn = document.getElementById('set-goal-btn');

setGoalBtn.addEventListener('click', () => {
  if (mode === 'goal') {
    cancelMode();
  } else {
    mode = 'goal';
    canvas_map.style.cursor = 'crosshair';
    setGoalBtn.classList.add('active'); // optional style
  }
});

canvas_map.addEventListener('click', (e) => {
  if (isDragging || mode !== 'goal') return;

  const rect = canvas_map.getBoundingClientRect();
  const clickX = e.clientX - rect.left - canvas_map.width / 2;
  const clickY = e.clientY - rect.top - canvas_map.height / 2;

  const mapX = (clickX - offsetX) / zoom;
  const mapY = (clickY - offsetY) / zoom;

  console.log(`🎯 Goal: (${mapX.toFixed(2)}, ${mapY.toFixed(2)})`);
  
  
  
  //window.electronAPI.sendNavigationGoal(mapX, mapY);

  cancelMode(); // ✅ ออกโหมดหลังคลิก
  goalPoint = { x: mapX, y: mapY };
  renderCanvas();
});

canvas_map.addEventListener('contextmenu', (e) => {
  if (mode !== 'none') {
    e.preventDefault(); // ❌ ไม่ให้เปิดเมนูขวา
    cancelMode();
  }
});

document.addEventListener('click', (e) => {
  if (
    mode === 'goal' &&
    !canvas_map.contains(e.target) &&
    !setGoalBtn.contains(e.target)
  ) {
    mode = 'none';
    canvas_map.style.cursor = 'default';
    setGoalBtn.classList.remove('active');
    renderCanvas();
  }
});

function cancelMode() {
  mode = 'none';
  canvas_map.style.cursor = 'default';
  setGoalBtn.classList.remove('active');
  goalPoint = null;
  renderCanvas();
}

document.getElementById('select-map-btn').addEventListener('click', async () => {
  if (!current_map_select.name) {
    alert("❗ No map selected");
    return;
  }

  activeMap = current_map_select;
  const mapName = activeMap.name;
  document.getElementById('active-map-name').textContent = mapName;

  await loadActiveMapMeta();   // ✅ โหลด resolution, origin, etc
  renderHomeCanvas();          // ✅ วาด canvas ใหม่จาก activeMap

  console.log(`📡 Selecting map "${mapName}" as active_map`);
  window.electronAPI.selectMap(mapName); // แจ้งฝั่ง main process
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
window.electronAPI.sendStopPatrol();
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

