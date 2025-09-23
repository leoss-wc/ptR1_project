// renderer.js

// ... (โค้ดส่วนบนทั้งหมด ตั้งแต่ line 1 ถึง 212) ...

// ⬇️⬇️⬇️ ส่วนที่แก้ไข ⬇️⬇️⬇️

// Import โมดูลใหม่ (ต้องแน่ใจว่า path ถูกต้อง)
import { PatrolManager } from './modules/patrolManager.js';

// 📦 Map Interface
const canvas_map = document.getElementById('staticMapCanvas');
let mapImage = null;
let zoom = 1.0;
let offsetX = 0;
let offsetY = 0;
let isDragging = false;
let dragStartX = 0;
let dragStartY = 0;

// --- State ที่เกี่ยข้องกับ Patrol ถูกย้ายออกไปแล้ว ---
// const patrolPath = []; // <-- ลบออก
// let goalPoint = null; // <-- ลบออก
let mode = 'none'; // 'none', 'goal', 'draw' (State ของ UI ยังเก็บไว้ที่นี่)

let current_map_select = { name: null, base64: null };
let activeMap = { name: null, base64: null };

// --- เริ่มต้นการทำงานของโมดูลใหม่ ---
PatrolManager.init();

// ... (โค้ดส่วน sync-maps-btn, onSyncComplete, loadLocalMaps, open-map-folder-btn, showOnCanvas... line 238-284) ...

function renderCanvas() {
  // ... (โค้ดส่วนต้นของ renderCanvas ... line 285-311) ...

  // --- 🟠 วาด path (ดึงข้อมูลจาก Manager) ---
  const path = PatrolManager.getPath(); // <-- ดึงข้อมูลจาก Manager
  if (path.length > 0) {
    ctx.strokeStyle = 'orange';
    ctx.lineWidth = 2;
    ctx.beginPath();

    path.forEach((pt, i) => { // <-- ใช้ path ที่ดึงมา
      const drawX = canvas_map.width / 2 + pt.x * zoom + offsetX;
      const drawY = canvas_map.height / 2 + pt.y * zoom + offsetY;
      if (i === 0) {
        ctx.moveTo(drawX, drawY);
      } else {
        ctx.lineTo(drawX, drawY);
      }
    });

    ctx.closePath();
    ctx.stroke();

    // 🔵 วาดจุด
    ctx.fillStyle = 'cyan';
    path.forEach((pt) => { // <-- ใช้ path ที่ดึงมา
      const drawX = canvas_map.width / 2 + pt.x * zoom + offsetX;
      const drawY = canvas_map.height / 2 + pt.y * zoom + offsetY;
      ctx.beginPath();
      ctx.arc(drawX, drawY, 4, 0, Math.PI * 2);
      ctx.fill();
    });
  }
  
  // --- 🎯 วาดจุด goal (ดึงข้อมูลจาก Manager) ---
  const goal = PatrolManager.getGoalPoint(); // <-- ดึงข้อมูลจาก Manager
  if (goal) { // <-- ใช้ goal ที่ดึงมา
    const drawX = canvas_map.width / 2 + goal.x * zoom + offsetX;
    const drawY = canvas_map.height / 2 + goal.y * zoom + offsetY;

    ctx.fillStyle = 'red';
    // ... (โค้ดวาดจุด goal ที่เหลือ ... line 348-358)
    ctx.stroke();
  }
}

// ... (โค้ด Zoom Controls ... line 360-413) ...

document.getElementById('clear-path-btn').addEventListener('click', () => {
  PatrolManager.clearPathAndGoal(); // <-- เรียกใช้ Manager
  cancelMode(); // (cancelMode ยังอยู่ในไฟล์นี้)
  console.log('Cleared Path and Point Goal');
});

// ... (โค้ด save-path-btn, Mouse drag ... line 420-457) ...

// ** แก้ไข: โค้ดเดิมของคุณมี click listener 2 อันสำหรับ canvas_map 
// ** อันแรกรอ 'isDrawMode' ซึ่งไม่เคยถูกตั้งค่า (น่าจะเป็นบั๊ก)
// ** ผมจะแก้ไขให้มันทำงานกับ mode = 'draw'

// *** (ต้องมีปุ่มสำหรับตั้ง mode 'draw' ใน HTML ของคุณ เช่น id="draw-path-btn") ***
const drawPathBtn = document.getElementById('draw-path-btn'); // (สมมติว่าคุณมีปุ่มนี้)

if(drawPathBtn) {
  drawPathBtn.addEventListener('click', () => {
    if (mode === 'draw') {
      cancelMode();
    } else {
      mode = 'draw';
      canvas_map.style.cursor = 'copy';
      drawPathBtn.classList.add('active');
      setGoalBtn.classList.remove('active'); // ปิดโหมดอื่น
    }
  });
}

// Listener สำหรับ "วาด Path" (เดิมคือ line 458)
canvas_map.addEventListener('click', (e) => {
  if (mode !== 'draw' || isDragging) return; // <-- แก้ไขให้เช็ค mode 'draw'

  const rect = canvas_map.getBoundingClientRect();
  const clickX = e.clientX - rect.left - canvas_map.width / 2;
  const clickY = e.clientY - rect.top - canvas_map.height / 2;

  const x = (clickX - offsetX) / zoom;
  const y = (clickY - offsetY) / zoom;

  PatrolManager.addPathPoint({ x, y }); // <-- เรียกใช้ Manager
  renderCanvas();
});

// ... (โค้ด window resize ... line 471) ...

const setGoalBtn = document.getElementById('set-goal-btn');

setGoalBtn.addEventListener('click', () => {
  if (mode === 'goal') {
    cancelMode();
  } else {
    mode = 'goal';
    canvas_map.style.cursor = 'crosshair';
    setGoalBtn.classList.add('active');
    if(drawPathBtn) drawPathBtn.classList.remove('active'); // ปิดโหมดอื่น
  }
});

// Listener สำหรับ "ตั้ง Goal" (เดิมคือ line 486)
canvas_map.addEventListener('click', (e) => {
  if (isDragging || mode !== 'goal') return;

  const rect = canvas_map.getBoundingClientRect();
  const clickX = e.clientX - rect.left - canvas_map.width / 2;
  const clickY = e.clientY - rect.top - canvas_map.height / 2;

  const mapX = (clickX - offsetX) / zoom;
  const mapY = (clickY - offsetY) / zoom;

  console.log(`🎯 Goal: (${mapX.toFixed(2)}, ${mapY.toFixed(2)})`);
  
  PatrolManager.setGoalPoint({ x: mapX, y: mapY }); // <-- เรียกใช้ Manager

  cancelMode(); // ✅ ออกโหมดหลังคลิก
  // goalPoint = { x: mapX, y: mapY }; // <-- ลบออก
  renderCanvas();
});

// ... (โค้ด contextmenu, document click ... line 505-524) ...

function cancelMode() {
  mode = 'none';
  canvas_map.style.cursor = 'default';
  setGoalBtn.classList.remove('active');
  if(drawPathBtn) drawPathBtn.classList.remove('active');
  
  PatrolManager.clearGoal(); // <-- เรียกใช้ Manager (เราจะลบแค่ goal, ไม่ลบ path)
  // goalPoint = null; // <-- ลบออก
  renderCanvas();
}

// ... (โค้ด select-map-btn, save-map-btn, onMapSaveResult ... line 533-555) ...

document.getElementById('start-patrol-btn').addEventListener('click', () => {
  PatrolManager.startPatrol(); // <-- เรียกใช้ Manager
});

// ปุ่ม Stop Patrol → ส่ง stop command
document.getElementById('stop-patrol-btn').addEventListener('click', () => {
  PatrolManager.stopPatrol(); // <-- เรียกใช้ Manager
});

// --- ลบออก ---
// window.electronAPI.onPatrolStatus((isMoving) => { ... }); // (ย้ายไป Manager)

// ... (โค้ด SLAM ... line 583-617) ...

// ... (โค้ด LiveMap ... line 620-718) ...

// --- ลบออก ---
// let currentPatrolIndex = 0; // (ย้ายไป Manager)
// let isPatrolling = false; // (ย้ายไป Manager)
// window.electronAPI.onPatrolStatus((isMoving) => { ... }); // (ย้ายไป Manager)
// function estimateCurrentIndex() { ... } // (ย้ายไป Manager)


document.getElementById('resume-patrol-btn').addEventListener('click', () => {
  PatrolManager.resumePatrol(); // <-- เรียกใช้ Manager
});

// ... (โค้ดที่เหลือ ... line 753 เป็นต้นไป) ...