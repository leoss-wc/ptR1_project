// modules/mapHome.js
// 👉 ใช้สำหรับแสดงแผนที่, หุ่นยนต์, goal ฯลฯ

import { activeMap } from './mapState.js';
import { robotPose,robotTrail } from './robotState.js';
import { goalPoint,isPatrolling , patrolPath} from './patrolState.js';
import { plannedPath } from './planState.js';

let canvas, ctx, mapImg;
let zoom = 1.0;
let offset = { x: 0, y: 0 };
let hasBeenReset = false;



export function initHomeMap(canvasElement) {
  // 1. ตั้งค่า Canvas และ Context
  canvas = canvasElement;
  ctx = canvas.getContext('2d');

  // 2. ตรวจสอบว่ามี Active Map ใน State หรือไม่
  if (activeMap.base64) {
    // 3. ถ้ามี ให้โหลดรูปภาพมาเตรียมไว้ใน mapImg ทันที
    setMapImage(activeMap.base64);
  } else {
    console.log("HomeMap: No active map to display on init.");
  }

  //: ResizeObserver จะเป็นตัวจัดการการ Reset View เริ่มต้น
  const resizeObserver = new ResizeObserver(() => {
    resizeCanvas();
  });
  resizeObserver.observe(canvas);
  initCanvasControls();
}


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
  // ถ้ายังไม่มี context หรือ canvas ยังไม่มีขนาด ให้หยุดทำงานทันที
  if (!ctx || canvas.width === 0 || canvas.height === 0) {
    console.warn(`HomeMap: Render skipped, canvas has no size yet (${canvas.width}x${canvas.height}).`);
    return;
  }
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  
  // 🔧 แก้ไข: เพิ่มเงื่อนไขตรวจสอบ Meta Data ที่นี่!
  // ต้องมีทั้ง mapImg และ activeMap.meta ก่อนถึงจะวาดอะไรลงไป
  if (mapImg && activeMap.meta) {
    ctx.drawImage(mapImg, offset.x, offset.y, mapImg.width * zoom, mapImg.height * zoom);
    
    // ฟังก์ชันวาดอื่นๆ จะถูกเรียกจากที่นี่ ซึ่งตอนนี้ปลอดภัยแล้ว
    drawRobotTrail(); 
    if (isPatrolling) {
      drawPatrolPath();
    } else {
      drawPlannedPath();
    }
    drawRobot();
    drawGoal();
  } else {
    // ✨ เพิ่ม: แสดงข้อความบอกสถานะถ้าแผนที่ยังไม่พร้อม
    ctx.fillStyle = 'gray';
    ctx.font = '16px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('Waiting for map data...', canvas.width / 2, canvas.height / 2);
  }
}



/*
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
*/

export function setMapImage(base64Str) {
  return new Promise((resolve) => {
    mapImg = new Image();
    // รีเซ็ต Flag ทุกครั้งที่เปลี่ยนแผนที่ใหม่
    hasBeenReset = false; 
    mapImg.onload = () => {
      console.log('🏠 HomeMap: Map image loaded successfully.');
      resizeCanvas();
      resolve();
    };
    mapImg.src =  base64Str;
  });
}

export function resetViewV2() {
  if (!canvas || !mapImg || canvas.width === 0 || canvas.height === 0) {
    console.warn(`HomeMap: resetViewV2 skipped, canvas has no size yet (${canvas.width}x${canvas.height}).`);
    return;
  }

  // 🔧 แก้ไข: เปลี่ยนมาใช้ Logic แบบ "Fit and Center"
  
  // 1. คำนวณอัตราส่วนการซูมที่พอดีกับความกว้างและความสูง
  const zoomX = canvas.width / mapImg.width;
  const zoomY = canvas.height / mapImg.height;

  // 2. ใช้ค่าซูมที่น้อยกว่า เพื่อให้แน่ใจว่าทั้งแผนที่อยู่ในกรอบ
  zoom = Math.min(zoomX, zoomY);

  // 3. จัดให้แผนที่อยู่กึ่งกลาง Canvas
  offset.x = (canvas.width - mapImg.width * zoom) / 2;
  offset.y = (canvas.height - mapImg.height * zoom) / 2;
  
  console.log(`🚀 HomeMap: View reset with "Fit and Center". New zoom=${zoom.toFixed(2)}`);

  renderDashboardMap();
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
  canvas.width = canvas.clientWidth;
  canvas.height = canvas.clientHeight;
  if (canvas.width > 0 && canvas.height > 0 && mapImg && !hasBeenReset) {
    resetViewV2();
    hasBeenReset = true;
  } else {
    renderDashboardMap();
  }
}

function drawPatrolPath() {
  if (patrolPath.length < 2 || !activeMap?.meta || !mapImg) return;
  // ✨ เพิ่ม: Log สำหรับตรวจสอบข้อมูลภายใน
  console.log("--- Debugging drawPatrolPath ---");
  console.log("First point in path:", patrolPath[0]);
  console.log("Origin from meta:", activeMap.meta.origin);
  console.log("---------------------------------");

  const { resolution, origin } = activeMap.meta;
  const imgH = mapImg.height;

  ctx.strokeStyle = 'orange'; // สีส้ม เหมือนกับในหน้า Static Map
  ctx.lineWidth = 2;
  ctx.setLineDash([5, 5]); // ทำให้เป็นเส้นประ เพื่อแยกความแตกต่าง
  ctx.beginPath();


  // ✨ DEBUG: เพิ่ม Log ตรวจสอบพิกัดที่คำนวณได้
  console.log(`--- Drawing on Canvas (Size: ${canvas.width}x${canvas.height}) ---`);


  patrolPath.forEach((point, index) => {
    // แปลง World Coordinate เป็น Screen Coordinate
    const px = (point.x - origin[0]) / resolution;
    const py = imgH - (point.y - origin[1]) / resolution;
    const screenX = px * zoom + offset.x;
    const screenY = py * zoom + offset.y;

    // ✨ DEBUG: แสดงค่าพิกัดของจุดแรกที่คำนวณได้
    if (index === 0) {
      console.log(`First point calculated at screen coordinates: (x: ${screenX.toFixed(2)}, y: ${screenY.toFixed(2)})`);
    }

    if (index === 0) {
      ctx.moveTo(screenX, screenY);
    } else {
      ctx.lineTo(screenX, screenY);
    }
  });
  ctx.stroke();
  ctx.setLineDash([]); // คืนค่าให้เป็นเส้นทึบสำหรับส่วนอื่น
}