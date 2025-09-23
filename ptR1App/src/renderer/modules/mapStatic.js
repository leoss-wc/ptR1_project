// modules/mapStatic.js
import { startPatrolState, stopPatrolState, patrolPath } from './patrolState.js';

import { activeMap } from './mapState.js';

import { setMapImage } from './mapHome.js';



let canvas, ctx;
let mapImage = null;
let zoom = 1.0;
let offsetX = 0;
let offsetY = 0;
let isDragging = false;
let isDrawing = false; // 👈 เพิ่มตัวแปรนี้
let dragStartX = 0;
let dragStartY = 0;

let current_map_select = { name: null, base64: null };
let goalPoint = null;
let mode = 'none';

export function initStaticMap() {
  canvas = document.getElementById('staticMapCanvas');
  ctx = canvas.getContext('2d');

  bindUI();
  setupCanvasEvents();
  loadLocalMapsToGallery(); // initial load
}

document.getElementById('select-map-btn').addEventListener('click', async () => {
  if (!current_map_select.name) {
    alert("❗ No map selected");
    return;
  }

  // --- อัปเดต Active Map State (เหมือนเดิม) ---
  activeMap.name = current_map_select.name;
  activeMap.base64 = current_map_select.base64; // base64 ที่ไม่มี 'data:image/...'
  document.getElementById('active-map-name').textContent = activeMap.name;
  console.log(`🗺️ Selected Map: ${activeMap.name}`);

  localStorage.setItem('activeMapName', activeMap.name);
  console.log(`💾 Saved active map '${activeMap.name}' to localStorage.`);

  // --- โหลด Meta Data (เหมือนเดิม) ---
  const result = await window.electronAPI.getMapMeta(activeMap.name);
  if (result.success) {
    activeMap.meta = result.data;
    console.log("✅ Loaded map meta:", result.data);
  } else {
    console.warn("❌ Failed to load map meta:", result.message);
    activeMap.meta = null; // Reset meta ถ้าโหลดไม่สำเร็จ
  }

  // ✅ 3. สั่งให้โมดูล mapHome วาดแผนที่ใหม่
  // เราจะส่งแค่ข้อมูล base64 ที่ไม่มีส่วนหัว 'data:image/png;base64,' ไป
  // เพราะฟังก์ชัน setMapImage จะเติมให้เอง
  if (activeMap.base64) {
    await setMapImage(activeMap.base64);
  }

  // --- สั่งให้ Main Process โหลดแผนที่ใน ROS (เหมือนเดิม) ---
  window.electronAPI.selectMap(activeMap.name);
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

function bindUI() {
  document.getElementById('zoom-in').addEventListener('click', () => {
    if (mapImage) {
      zoom = Math.min(zoom * 1.2, 10);
      renderCanvas();
    }
  });

  document.getElementById('zoom-out').addEventListener('click', () => {
    if (mapImage) {
      zoom = Math.max(zoom / 1.2, 0.1);
      renderCanvas();
    }
  });

  document.getElementById('reset-view').addEventListener('click', () => {
    zoom = 1.0;
    offsetX = 0;
    offsetY = 0;
    renderCanvas();
  });

  document.getElementById('clear-path-btn').addEventListener('click', () => {
    patrolPath.length = 0;
    goalPoint = null;
    cancelMode();
    renderCanvas();
    console.log('📍 Cleared Path and Goal');
  });

  document.getElementById('save-path-btn').addEventListener('click', () => {
    console.log('💾 Saved Path:', patrolPath);
    window.electronAPI.sendPatrolPath(patrolPath);
  });

  document.getElementById('start-patrol-btn').addEventListener('click', startPatrol);
  document.getElementById('stop-patrol-btn').addEventListener('click', stopPatrol);


  document.getElementById('sync-maps-btn').addEventListener('click', () => {
    window.electronAPI.syncMaps();
  });

  document.getElementById('set-goal-btn').addEventListener('click', () => {
    if (mode === 'goal') {
      cancelMode();
      return;
    }
    
    //ล้างข้อมูลของอีกฝั่ง (Draw Path)
    patrolPath.length = 0;
    console.log('Cleared Patrol Path');
    
    //Reset UI ทั้งหมดก่อน
    cancelMode(); 
    
    //ตั้งค่าโหมด Goal
    mode = 'goal';
    canvas.style.cursor = 'crosshair';
    document.getElementById('set-goal-btn').classList.add('active');
    
    renderCanvas();
  });

  const drawModeBtn = document.getElementById('toggle-draw-mode');
  drawModeBtn.addEventListener('click', () => {
    if (mode === 'draw') {
      cancelMode();
      return;
    }
    
    //ล้างข้อมูลของอีกฝั่ง (Goal Point)
    goalPoint = null;
    console.log('Cleared Goal Point');

    //Reset UI ทั้งหมดก่อน
    cancelMode();

    //ตั้งค่าโหมด Draw
    mode = 'draw';
    canvas.style.cursor = 'crosshair';
    drawModeBtn.textContent = 'Draw :ON';
    drawModeBtn.classList.add('active');
    
    //วาด Canvas ใหม่เพื่อให้ Goal Marker ที่ลบหายไป
    patrolPath.length = 0; // เคลียร์ path เก่าเพื่อเริ่มวาดใหม่ (ของเดิม)
    renderCanvas();
    });

  window.electronAPI.onSyncComplete((mapList) => {
    const gallery = document.getElementById('map-gallery');
    gallery.innerHTML = '';
    mapList.forEach(({ name, base64 }) => addMapToGallery(name, base64));
    loadLocalMapsToGallery();
  });
}

function setupCanvasEvents() {
   canvas.addEventListener('mousedown', (e) => {
    if (mode === 'draw') {
      isDrawing = true;
      addPathPoint(e); // เพิ่มจุดแรกเมื่อคลิก
      renderCanvas();
    } else {
      isDragging = true;
      dragStartX = e.clientX;
      dragStartY = e.clientY;
    }
  });

  canvas.addEventListener('mouseup', () => {
    isDrawing = false; // หยุดวาดเมื่อปล่อยเมาส์
    isDragging = false;
  });

  canvas.addEventListener('mouseleave', () => {
    isDrawing = false; // หยุดวาดเมื่อเมาส์ออกจาก Canvas
    isDragging = false;
  });

  canvas.addEventListener('mousemove', (e) => {

    if (isDrawing) {
      addPathPoint(e); 
      renderCanvas();
      return; 
    }

    if (!isDragging) return;
    const dx = e.clientX - dragStartX;
    const dy = e.clientY - dragStartY;
    dragStartX = e.clientX;
    dragStartY = e.clientY;

    offsetX += dx;
    offsetY += dy;
    renderCanvas();
  });

    
  canvas.addEventListener('mousedown', (e) => {
    isDragging = true;
    dragStartX = e.clientX;
    dragStartY = e.clientY;
  });

  canvas.addEventListener('mouseup', () => isDragging = false);
  canvas.addEventListener('mouseleave', () => isDragging = false);

  canvas.addEventListener('mousemove', (e) => {
    if (!isDragging) return;
    const dx = e.clientX - dragStartX;
    const dy = e.clientY - dragStartY;
    dragStartX = e.clientX;
    dragStartY = e.clientY;

    offsetX += dx;
    offsetY += dy;
    renderCanvas();
  });

  canvas.addEventListener('wheel', (e) => {
    if (!mapImage) return;
    e.preventDefault();
    const zoomFactor = 1.1;
    const oldZoom = zoom;

    const rect = canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left - canvas.width / 2;
    const my = e.clientY - rect.top - canvas.height / 2;

    zoom = e.deltaY < 0
      ? Math.min(zoom * zoomFactor, 10)
      : Math.max(zoom / zoomFactor, 0.1);

    const scale = zoom / oldZoom;
    offsetX = mx - (mx - offsetX) * scale;
    offsetY = my - (my - offsetY) * scale;

    renderCanvas();
  });

  canvas.addEventListener('click', (e) => {
    if (isDragging || mode !== 'goal') return;

    const rect = canvas.getBoundingClientRect();
    const clickX = e.clientX - rect.left - canvas.width / 2;
    const clickY = e.clientY - rect.top - canvas.height / 2;

    const mapX = (clickX - offsetX) / zoom;
    const mapY = (clickY - offsetY) / zoom;

    goalPoint = { x: mapX, y: mapY };
    console.log('🎯 Goal Point:', goalPoint);
    renderCanvas();
    cancelMode();
  });

  window.addEventListener('resize', () => renderCanvas());

  canvas.addEventListener('contextmenu', (e) => {
    if (mode !== 'none') {
      e.preventDefault();
      cancelMode();
    }
  });

  document.addEventListener('click', (e) => {
    if (
      mode === 'goal' &&
      !canvas.contains(e.target) &&
      !document.getElementById('set-goal-btn').contains(e.target)
    ) {
      cancelMode();
    }
  });
}

function cancelMode() {
  mode = 'none';
  isDrawing = false;
  canvas.style.cursor = 'default';

  // อัปเดตปุ่ม "Set Goal"
  document.getElementById('set-goal-btn').classList.remove('active');
  
  // อัปเดตปุ่ม "Draw"
  const drawModeBtn = document.getElementById('toggle-draw-mode');
  drawModeBtn.textContent = 'Draw :OFF';
  drawModeBtn.classList.remove('active');
  
  renderCanvas();
}

function addPathPoint(e) {
  const rect = canvas.getBoundingClientRect();
  const clickX = e.clientX - rect.left - canvas.width / 2;
  const clickY = e.clientY - rect.top - canvas.height / 2;

  // แปลงพิกัดบนหน้าจอเป็นพิกัดบนแผนที่ (คำนึงถึง zoom/pan)
  const mapX = (clickX - offsetX) / zoom;
  const mapY = (clickY - offsetY) / zoom;

  // สร้าง object ของจุดใหม่เพื่อง่ายต่อการใช้งาน
  const newPoint = { x: mapX, y: mapY };

  console.log(`📍 Added point: X=${newPoint.x.toFixed(3)}, Y=${newPoint.y.toFixed(3)}`);

  patrolPath.push({ x: mapX, y: mapY });
  console.log('📜 Current full path:', patrolPath);
}

function renderCanvas() {
  if (!mapImage) return;
  canvas.width = canvas.clientWidth;
  canvas.height = canvas.clientHeight;
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const imgW = mapImage.width * zoom;
  const imgH = mapImage.height * zoom;
  const cx = canvas.width / 2;
  const cy = canvas.height / 2;

  ctx.drawImage(
    mapImage,
    cx - imgW / 2 + offsetX,
    cy - imgH / 2 + offsetY,
    imgW,
    imgH
  );

  // 🔶 วาดชื่อแผนที่
  if (current_map_select.name) {
    ctx.font = '16px sans-serif';
    ctx.fillStyle = 'white';
    ctx.fillText(`Map: ${current_map_select.name}`, 10, 10);
  }

  // 🟠 วาด path
  if (patrolPath.length > 0) {
    ctx.strokeStyle = 'orange';
    ctx.lineWidth = 2;
    ctx.beginPath();
    patrolPath.forEach((pt, i) => {
      const x = cx + pt.x * zoom + offsetX;
      const y = cy + pt.y * zoom + offsetY;
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.closePath();
    ctx.stroke();

    ctx.fillStyle = 'cyan';
    patrolPath.forEach((pt) => {
      const x = cx + pt.x * zoom + offsetX;
      const y = cy + pt.y * zoom + offsetY;
      ctx.beginPath();
      ctx.arc(x, y, 4, 0, Math.PI * 2);
      ctx.fill();
    });
  }

  // 🎯 goal point
  if (goalPoint) {
    const x = cx + goalPoint.x * zoom + offsetX;
    const y = cy + goalPoint.y * zoom + offsetY;
    ctx.beginPath();
    ctx.arc(x, y, 6, 0, 2 * Math.PI);
    ctx.fillStyle = 'red';
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = 'white';
    ctx.stroke();
  }
}

function loadLocalMapsToGallery() {
  window.electronAPI.getLocalMaps().then((maps) => {
    const gallery = document.getElementById('map-gallery');
    gallery.innerHTML = '';
    maps.forEach(({ name, base64 }) => addMapToGallery(name, base64));
  });
}

function addMapToGallery(name, base64) {
  const img = document.createElement('img');
  img.src = base64;
  img.alt = name;
  img.title = name;
  img.className = 'map-thumb';
  img.style.cursor = 'pointer';

  img.addEventListener('click', () => {
    mapImage = new Image();
    mapImage.onload = renderCanvas;
    mapImage.src = base64;
    current_map_select = { name, base64 };
  });

  document.getElementById('map-gallery').appendChild(img);
}

function startPatrol() {
  if (patrolPath.length < 2) {
    console.warn('⚠️ Path is too short to start patrol. Please draw a longer path.');
    alert('Please draw a path with at least 2 points.');
    return;
  }
  
  console.log('▶️ Starting patrol with path:', patrolPath);
  
  // อัปเดต State ส่วนกลาง
  startPatrolState(patrolPath);
  
  //ส่ง path ทั้งหมดไปให้ Backend/Robot
  window.electronAPI.sendPatrolPath(patrolPath); 
  
  // log เป้าหมายแรก
  const currentGoal = patrolPath[0];
  console.log(`🏁 First goal is: X=${currentGoal.x.toFixed(3)}, Y=${currentGoal.y.toFixed(3)}`);
}

/**
 * ฟังก์ชันสำหรับหยุดการลาดตระเวน (Patrol)
 */
function stopPatrol() {
  console.log('🛑 Stopping patrol...');

  // อัปเดต State ส่วนกลาง
  stopPatrolState();

  // ในการใช้งานจริง จะส่งคำสั่งหยุดไปให้ Backend/Robot
  window.electronAPI.sendStopPatrol();
}