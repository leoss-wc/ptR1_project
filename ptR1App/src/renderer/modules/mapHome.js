// modules/mapHome.js
//ใช้สำหรับแสดงแผนที่, หุ่นยนต์, goal ฯลฯ

import { activeMap } from './mapState.js';
import { robotPose,robotTrail } from './robotState.js';
import { goalPoint,isPatrolling , patrolPath} from './patrolState.js';
import { plannedPath } from './planState.js';
import { latestScan } from './laserScanState.js';
import { getYawFromQuaternion } from './utils.js';

let canvas, ctx, mapImg;
let zoom = 1.0;
let offset = { x: 0, y: 0 };
let hasBeenReset = false;
let animationFrameId = null;
const TARGET_FPS = 25;            // เป้าหมาย: 20 เฟรมต่อวินาที (ปรับเลขนี้ได้ตามใจ)
const FRAME_INTERVAL = 1000 / TARGET_FPS; // คำนวณเป็นมิลลิวินาทีต่อเฟรม
let isMapLoading = false;
let lastTime = 0; // ตัวแปรเก็บเวลาล่าสุดที่วาด

export function initHomeMap(canvasElement) {
  canvas = canvasElement;
  ctx = canvas.getContext('2d');

  if (activeMap.base64) {
    setMapImage(activeMap.base64);
  }
  else {
    console.warn("HomeMap: No map image data available at initialization.");
  }

  const resizeObserver = new ResizeObserver(() => {
    resizeCanvas();
  });
  resizeObserver.observe(canvas);
  initCanvasControls();

  //เริ่มวาดต่อเนื่องทันทีที่ Init
  startRenderLoop();
}
//เริ่ม Loop
export function startRenderLoop() {
  if (animationFrameId) return;
  
  const loop = (timestamp) => {
    if (canvas.width === 0 || canvas.height === 0) {
        resizeCanvas();
    }
    if (!mapImg && activeMap.base64 && !isMapLoading) {
        console.log("HomeMap: Found new map data! Loading...");
        isMapLoading = true;
        setMapImage(activeMap.base64).then(() => {
            isMapLoading = false;
        });
    }
    // คำนวณเวลาที่ผ่านไปตั้งแต่เฟรมที่แล้ว
    const elapsed = timestamp - lastTime;

    // ถ้าเวลาผ่านไปมากกว่าที่กำหนด (เช่น เกิน 50ms สำหรับ 20FPS) ถึงจะยอมให้วาด
    if (elapsed > FRAME_INTERVAL) {
      renderDashboardMap();
      // อัปเดตเวลาล่าสุด (ลบส่วนเกินออกเพื่อให้จังหวะคงที่)
      lastTime = timestamp - (elapsed % FRAME_INTERVAL);
    }
    // วนลูปต่อไป (browser จะเรียกฟังก์ชันนี้เรื่อยๆ แต่เราจะวาดแค่ตอนถึงเวลา)
    animationFrameId = requestAnimationFrame(loop);
  };
  animationFrameId = requestAnimationFrame(loop);
  console.log(`HomeMap: Render loop started at ~${TARGET_FPS} FPS.`);
}

// หยุด Loop เมื่อเปลี่ยนหน้า
export function stopRenderLoop() {
  if (animationFrameId) {
    cancelAnimationFrame(animationFrameId);
    animationFrameId = null;
    console.log("HomeMap: Render loop stopped.");
  }
}

function drawRobot() {
  if (!robotPose?.position || !activeMap?.meta || !mapImg){
    return;
  } 

  const { resolution, origin } = activeMap.meta;
  const imgH = mapImg.height;

  // 1. แปลง World Coordinate เป็นพิกัดพิกเซลบนแผนที่
  const px = (robotPose.position.x - origin[0]) / resolution;
  const py = imgH - (robotPose.position.y - origin[1]) / resolution;

  // 2. แปลงเป็นพิกัดหน้าจอ (บวกการเลื่อน Offset และการซูม)
  const screenX = px * zoom + offset.x;
  const screenY = py * zoom + offset.y;

  // 3. ดึงมุม Yaw
  const yaw = getYawFromQuaternion(robotPose.orientation);

  ctx.save();
  ctx.translate(screenX, screenY); // ย้ายจุดศูนย์กลางการวาดไปที่ตัวหุ่น
  ctx.rotate(-yaw);                // หมุนตามทิศทางหุ่น (ROS ใช้มุมทวนเข็มนาฬิกาเป็นบวก)

  // 4. วาดรูปสามเหลี่ยม (หัวชี้ไปทางแกน +X ทิศตะวันออก)
  ctx.beginPath();
  ctx.moveTo(12, 0);     // จุดแหลม (หัวหุ่น)
  ctx.lineTo(-6, -6);    // ท้ายหุ่นฝั่งซ้าย
  ctx.lineTo(-6, 6);     // ท้ายหุ่นฝั่งขวา
  ctx.closePath();
  
  // ลงสีแดงโปร่งแสง
  ctx.fillStyle = 'rgba(255, 0, 0, 0.8)';
  ctx.fill();
  
  //ใส่ขอบสีดำบางๆ ให้ตัวหุ่นดูโดดเด่นขึ้นเวลาวิ่งบนแผนที่สว่าง
  ctx.strokeStyle = '#000000';
  ctx.lineWidth = 1;
  ctx.stroke();

  ctx.restore();
}

function drawGoal() {
  if (!goalPoint?.position || !activeMap?.meta || !mapImg) {
    return;
  }
  const { resolution, origin } = activeMap.meta;
  const imgH = mapImg.height;

  const { position, orientation } = goalPoint;

  const px = (position.x - origin[0]) / resolution;
  const py = imgH - (position.y - origin[1]) / resolution;
  const screenX = px * zoom + offset.x;
  const screenY = py * zoom + offset.y;

  ctx.beginPath();
  ctx.arc(screenX, screenY, 5, 0, 2 * Math.PI);
  ctx.fillStyle = 'red';
  ctx.fill();
  if (orientation) {
    const yaw = getYawFromQuaternion(orientation);
    const arrowLength = 15;

    ctx.save();
    ctx.translate(screenX, screenY);
    ctx.rotate(-yaw); // หมุนตามทิศทางของ Goal
    ctx.beginPath();
    ctx.moveTo(arrowLength, 0);
    ctx.lineTo(0, -5);
    ctx.lineTo(0, 5);
    ctx.closePath();
    ctx.fillStyle = 'red';
    ctx.fill();
    ctx.restore();
  }
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
  ctx.imageSmoothingEnabled = false;
  
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
    drawLaserScan();
  } else {
    ctx.fillStyle = 'gray';
    ctx.font = '16px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('Waiting for map data...', canvas.width / 2, canvas.height / 2);
  }
}

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

  // Logic แบบ "Fit and Center"
  // 1. คำนวณอัตราส่วนการซูมที่พอดีกับความกว้างและความสูง
  const zoomX = canvas.width / mapImg.width;
  const zoomY = canvas.height / mapImg.height;

  // 2. ใช้ค่าซูมที่น้อยกว่า เพื่อให้แน่ใจว่าทั้งแผนที่อยู่ในกรอบ
  zoom = Math.min(zoomX, zoomY);

  // 3. จัดให้แผนที่อยู่กึ่งกลาง Canvas
  offset.x = (canvas.width - mapImg.width * zoom) / 2;
  offset.y = (canvas.height - mapImg.height * zoom) / 2;
  
  console.log(`HomeMap: View reset with "Fit and Center". New zoom=${zoom.toFixed(2)}`);

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
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;

  // ถ้าขนาดยังเป็น 0 (เช่น ยังซ่อนอยู่) ให้จบการทำงานไปก่อน อย่าเพิ่งทำอะไร
  if (width === 0 || height === 0) return;

  if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
      if (mapImg && !hasBeenReset) {
        resetViewV2();
        hasBeenReset = true;
      } else {
        renderDashboardMap();
      }
      console.log(`HomeMap: Resized to ${width}x${height}`);
  }
}
function drawPatrolPath() {
  // เปลี่ยนเป็น < 1 เพื่อให้แม้มีแค่จุดเดียวก็ยังวาดโชว์บนหน้า Home ได้
  if (patrolPath.length < 1 || !activeMap?.meta || !mapImg) return; 

  const { resolution, origin } = activeMap.meta;
  const imgH = mapImg.height;

  // --- 1. วาดเส้นประเชื่อมแต่ละจุด (ถ้ามีมากกว่า 1 จุด) ---
  if (patrolPath.length > 1) {
    ctx.strokeStyle = 'orange'; // สีส้ม
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 5]); // ทำให้เป็นเส้นประ
    ctx.beginPath();

    patrolPath.forEach((point, index) => {
      // แปลง World Coordinate เป็น Screen Coordinate
      const px = (point.x - origin[0]) / resolution;
      const py = imgH - (point.y - origin[1]) / resolution;
      const screenX = px * zoom + offset.x;
      const screenY = py * zoom + offset.y;
      
      if (index === 0) {
        ctx.moveTo(screenX, screenY);
      } else {
        ctx.lineTo(screenX, screenY);
      }
    });
    ctx.stroke();
    ctx.setLineDash([]); // คืนค่าให้เป็นเส้นทึบสำหรับส่วนอื่น
  }

  // --- 2. วาดจุด Waypoint แต่ละจุด (วงกลม) ---
  patrolPath.forEach((point, index) => {
    // แปลงพิกัดอีกรอบสำหรับวาดวงกลม
    const px = (point.x - origin[0]) / resolution;
    const py = imgH - (point.y - origin[1]) / resolution;
    const screenX = px * zoom + offset.x;
    const screenY = py * zoom + offset.y;
    
    // ตั้งค่าขนาดจุด (รัศมี 4 พิกเซล)
    const radius = 4;
    
    ctx.beginPath();
    ctx.arc(screenX, screenY, radius, 0, 2 * Math.PI);
    
    // สีของจุด: จุดแรก (Start) สีเขียว, จุดอื่นๆ สีฟ้า (Cyan) ให้เหมือนหน้า Static
    ctx.fillStyle = (index === 0) ? '#00FF00' : 'cyan'; 
    ctx.fill();
    
    // ตัดขอบดำบางๆ ให้จุดดูมีมิติและมองเห็นชัดขึ้นเวลาอยู่บนพื้นสีสว่าง
    ctx.strokeStyle = '#000000';
    ctx.lineWidth = 1;
    ctx.stroke();
  });
}

function drawLaserScan() {
  if (!latestScan || !robotPose.position || !activeMap.meta || !mapImg) return;

  const { resolution, origin } = activeMap.meta;
  const imgH = mapImg.height;
  const robotYaw = getYawFromQuaternion(robotPose.orientation);

  ctx.fillStyle = 'rgba(255, 0, 255, 0.7)';

  for (let i = 0; i < latestScan.ranges.length; i++) {
    const range = latestScan.ranges[i];
    
    // กรองระยะที่ผิดพลาดออก
    if (range < 0.1 || range > 10.0) continue; 

    const angle = latestScan.angle_min + i * latestScan.angle_increment;
    const totalAngle = robotYaw + angle;

    // คำนวณตำแหน่งของจุดเลเซอร์ใน World Coordinates
    const worldX = robotPose.position.x + range * Math.cos(totalAngle);
    const worldY = robotPose.position.y + range * Math.sin(totalAngle);

    // แปลง World Coordinates เป็น Screen Coordinates
    const scanPx = (worldX - origin[0]) / resolution;
    const scanPy = imgH - (worldY - origin[1]) / resolution;
    const screenX = scanPx * zoom + offset.x;
    const screenY = scanPy * zoom + offset.y;
    ctx.fillRect(screenX, screenY, 2, 2); // วาดสี่เหลี่ยมขนาด 2x2 pixels
  }
}
