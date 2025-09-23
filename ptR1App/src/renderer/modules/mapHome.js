// modules/mapHome.js
// 👉 ใช้สำหรับแสดงแผนที่, หุ่นยนต์, goal ฯลฯ

import { activeMap } from './mapState.js';
import { robotPose,robotTrail } from './robotState.js';
import { goalPoint,isPatrolling } from './patrolState.js';
import { plannedPath } from './planState.js';

let canvas, ctx, mapImg;
let zoom = 1.0;
let offset = { x: 0, y: 0 };


function getYawFromQuaternion(q) {
  const { x, y, z, w } = q;
  return Math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

function drawRobot() {
  if (!robotPose?.position || !activeMap?.meta || !mapImg){
    //console.log("mapHome: No robotPose.");
    return;
  } 

  const { resolution, origin } = activeMap.meta;
  const imgH = mapImg.height;

  const px = (robotPose.position.x - origin[0]) / resolution;
  const py = imgH - (robotPose.position.y - origin[1]) / resolution;

  const yaw = getYawFromQuaternion(robotPose.orientation);
  const screenX = px * zoom + offset.x;
  const screenY = py * zoom + offset.y;

  ctx.save();
  ctx.translate(screenX, screenY);
  ctx.rotate(-yaw);
  ctx.beginPath();
  ctx.moveTo(0, -10);
  ctx.lineTo(7, 10);
  ctx.lineTo(-7, 10);
  ctx.closePath();
  ctx.fillStyle = 'lime';
  ctx.fill();
  ctx.restore();
}

function drawGoal() {
  if (!goalPoint || !activeMap?.meta || !mapImg) {
    //console.log("mapHome: No goalPoint.");
    return;
  }
  const { resolution, origin } = activeMap.meta;
  const imgH = mapImg.height;

  const px = (goalPoint.x - origin[0]) / resolution;
  const py = imgH - (goalPoint.y - origin[1]) / resolution;

  const screenX = px * zoom + offset.x;
  const screenY = py * zoom + offset.y;

  ctx.beginPath();
  ctx.arc(screenX, screenY, 5, 0, 2 * Math.PI);
  ctx.fillStyle = 'red';
  ctx.fill();
}

function drawRobotTrail() {
  if (robotTrail.length < 2) return;

  const { resolution, origin } = activeMap.meta;
  const imgH = mapImg.height;

  ctx.strokeStyle = 'rgba(0, 255, 255, 0.5)'; // สีฟ้าโปร่งแสง
  ctx.lineWidth = 2;
  ctx.beginPath();

  robotTrail.forEach((pos, index) => {
    const px = (pos.x - origin[0]) / resolution;
    const py = imgH - (pos.y - origin[1]) / resolution;
    const screenX = px * zoom + offset.x;
    const screenY = py * zoom + offset.y;

    if (index === 0) {
      ctx.moveTo(screenX, screenY);
    } else {
      ctx.lineTo(screenX, screenY);
    }
  });
  ctx.stroke();
}
function drawPlannedPath() {
  if (plannedPath.length < 2) return;

  const { resolution, origin } = activeMap.meta;
  const imgH = mapImg.height;

  ctx.strokeStyle = 'rgba(255, 165, 0, 0.8)'; // สีส้ม
  ctx.lineWidth = 2;
  ctx.beginPath();
  plannedPath.forEach((pos, index) => {
    const px = (pos.x - origin[0]) / resolution;
    const py = imgH - (pos.y - origin[1]) / resolution;
    const screenX = px * zoom + offset.x;
    const screenY = py * zoom + offset.y;
    index === 0 ? ctx.moveTo(screenX, screenY) : ctx.lineTo(screenX, screenY);
  });
  ctx.stroke();
}

export function renderDashboardMap() {
  if (!ctx) return;

  // ล้างหน้าจอ
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  
  // ถ้ามีรูปภาพแล้ว ค่อยวาด
  if (mapImg) {
    ctx.drawImage(mapImg, offset.x, offset.y, mapImg.width * zoom, mapImg.height * zoom);

    // วาดเส้นทางที่ผ่านมาแล้วเสมอ
    drawRobotTrail(); 

    // ตรวจสอบสถานะเพื่อวาดเส้นทางข้างหน้า
    if (isPatrolling) {
      drawPatrolPath(); // 👈 วาดเส้นทาง Patrol ทั้งหมด (อาจจะต้องย้ายฟังก์ชันนี้มา)
    } else {
      drawPlannedPath(); // 👈 วาดเส้นทางที่ Navigator คำนวณ
    }

    drawRobotTrail();
    drawRobot();
    drawGoal();
  }
}

export function setupMapCanvas(canvasElement) {
  canvas = canvasElement;
  ctx = canvas.getContext('2d');

  const resizeObserver = new ResizeObserver(() => {
    resizeCanvas();
  });
  resizeObserver.observe(canvas);

  initCanvasControls();
  //renderLoop();
}

export function setMapImage(base64Str) {
  return new Promise((resolve) => {
    mapImg = new Image();
    mapImg.onload = () => {
      resetViewV2();
      renderDashboardMap();
      resolve();
    };
    mapImg.src =  base64Str;
  });
}

export function resetViewV2() {
  if (!canvas || !mapImg) return;

  // 1. คำนวณ zoom พื้นฐานเพื่อให้กว้างพอดีกับ Canvas (เหมือนเดิม)
  const baseZoom = canvas.width / mapImg.width;

  // 2. ✅ เพิ่มตัวคูณเพื่อซูมเข้าไปใกล้กว่าเดิม
  const initialZoomMultiplier = 5; // Multiply zoom control  
  zoom = baseZoom * initialZoomMultiplier;

  // 3. จัดให้แผนที่อยู่กึ่งกลาง
  offset.x = (canvas.width - mapImg.width * zoom) / 2;
  offset.y = (canvas.height - mapImg.height * zoom) / 2;

  //console.log(`View reset (Multiplied): zoom=${zoom}, offset=`, offset);
}

function initCanvasControls() {
  if (!canvas) return;

  let isDragging = false;
  let lastX, lastY;

  // --- 1. ควบคุมการ Pan (ลากเมาส์) ---
  canvas.addEventListener('mousedown', (e) => {
    isDragging = true;
    lastX = e.clientX;
    lastY = e.clientY;
    canvas.style.cursor = 'grabbing';
    renderDashboardMap();
  });

  canvas.addEventListener('mousemove', (e) => {
    if (!isDragging) return;
    const dx = e.clientX - lastX;
    const dy = e.clientY - lastY;
    offset.x += dx;
    offset.y += dy;
    lastX = e.clientX;
    lastY = e.clientY;
    renderDashboardMap();
  });

  canvas.addEventListener('mouseup', () => {
    isDragging = false;
    canvas.style.cursor = 'grab';
  });
  
  canvas.addEventListener('mouseleave', () => {
    isDragging = false;
    canvas.style.cursor = 'default';
  });

  // --- 2. ควบคุมการ Zoom (Scroll wheel) ---
  canvas.addEventListener('wheel', (e) => {
    e.preventDefault(); // ป้องกันหน้าเว็บเลื่อน

    // --- 1. หาตำแหน่งของเมาส์เทียบกับ Canvas ---
    const rect = canvas.getBoundingClientRect();
    const mouseX = e.clientX - rect.left;
    const mouseY = e.clientY - rect.top;

    // --- 2. แปลงตำแหน่งเมาส์บนหน้าจอ ให้เป็นพิกัดบนแผนที่ (ก่อนซูม) ---
    const mapXBeforeZoom = (mouseX - offset.x) / zoom;
    const mapYBeforeZoom = (mouseY - offset.y) / zoom;

    // --- 3. คำนวณค่า zoom ใหม่ ---
    const zoomFactor = e.deltaY < 0 ? 1.1 : 0.9; // Scroll ขึ้น = ซูมเข้า, Scroll ลง = ซูมออก
    const newZoom = zoom * zoomFactor;
    
    // จำกัดค่า zoom ไม่ให้มากหรือน้อยเกินไป
    zoom = Math.max(0.1, Math.min(newZoom, 20)); 

    // --- 4. คำนวณ offset ใหม่เพื่อตรึงตำแหน่งเมาส์ไว้ที่เดิม ---
    offset.x = mouseX - mapXBeforeZoom * zoom;
    offset.y = mouseY - mapYBeforeZoom * zoom;

    renderDashboardMap(); 
  });
}

function resizeCanvas() {
  if (!canvas) return;

  // อ่านขนาดที่แสดงผลจริง แล้วกำหนดให้เป็นขนาดของพื้นที่วาด
  canvas.width = canvas.clientWidth;
  canvas.height = canvas.clientHeight;

  // สั่งวาดใหม่เพื่อให้การเปลี่ยนแปลงมีผลทันที
  renderDashboardMap(); 
}