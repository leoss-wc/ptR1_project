// src/renderer/modules/mapLive.js
import { applyTransform, restoreTransform } from './mapView.js';
import * as mapView from './mapView.js';

// สร้าง Canvas จำลอง (Off-screen canvas) เพื่อเก็บภาพแผนที่ดิบ
let canvas;
const offscreenCanvas = document.createElement('canvas');
const offscreenCtx = offscreenCanvas.getContext('2d');
let latestRobotPose = null;
let currentMapInfo = null;
let isLiveMapReady = false;
let isLiveMapInitialized = false;

// เตรียมค่าสีไว้ล่วงหน้า (ABGR Format สำหรับ Little Endian Systems)
// 0xFF = 255
const COLOR_UNKNOWN = 0xFF808080; // Gray (128, 128, 128, 255)
const COLOR_FREE = 0xFFFFFFFF;    // White (255, 255, 255, 255)
const COLOR_OCCUPIED = 0xFF000000; // Black (0, 0, 0, 255)


export function processLiveMapData(mapData) {
    if (!mapData || !mapData.info || !mapData.data) return;

    currentMapInfo = mapData.info;

    const hasMeaningfulData = mapData.data.some(value => value !== -1);
    if (hasMeaningfulData) isLiveMapReady = true;

    const width = mapData.info.width;
    const height = mapData.info.height;

    if (offscreenCanvas.width !== width || offscreenCanvas.height !== height) {
        offscreenCanvas.width = width;
        offscreenCanvas.height = height;
    }

    const imageData = offscreenCtx.createImageData(width, height);
    const buf32 = new Uint32Array(imageData.data.buffer);


    for (let i = 0; i < mapData.data.length; i++) {
        const val = mapData.data[i];
        if (val === -1) {
            buf32[i] = COLOR_UNKNOWN;
        } else if (val === 0) {
            buf32[i] = COLOR_FREE;
        } else {
            buf32[i] = COLOR_OCCUPIED;
        }
    }
    offscreenCtx.putImageData(imageData, 0, 0);
}

function quaternionToYaw(q) {
  return Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

function drawRobotOnLiveMap(ctx) {

    if (!latestRobotPose) {
        console.warn("No robot pose  available to draw robot.");
        return;         
    }else if(!currentMapInfo){
        console.warn("No map info available to draw robot.");
        return;
    };

    const { resolution, origin, height } = currentMapInfo;
    const pose = latestRobotPose;
  
    // คำนวณพิกัด Pixel (กลับแกน Y ตามมาตรฐาน Map Server)
    const px = (pose.position.x - origin.position.x) / resolution;
    const py = height - ((pose.position.y - origin.position.y) / resolution);

    

    // วาดตัวหุ่นยนต์ (วงกลมสีฟ้า)
    ctx.beginPath();
    ctx.arc(px, py, 5, 0, 2 * Math.PI, false);
    ctx.fillStyle = 'rgba(0, 150, 255, 0.8)';
    ctx.fill();
    ctx.lineWidth = 1;
    ctx.strokeStyle = '#FFFFFF';
    ctx.stroke();

    // วาดลูกศรบอกทิศทาง
    const yaw = quaternionToYaw(pose.orientation);
    const arrowLength = 15;
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.lineTo(px + arrowLength * Math.cos(-yaw), py + arrowLength * Math.sin(-yaw));
    ctx.strokeStyle = '#FFFFFF';
    ctx.lineWidth = 2;
    ctx.stroke();
}
export function updateLiveRobotPose(pose) {
  latestRobotPose = pose;
  //console.log(`Update robot pose: x=${pose.position.x}, y=${pose.position.y}`);
}

// ฟังก์ชันเริ่มต้นสำหรับ Live Map
export function initLiveMap() {
  canvas = document.getElementById('liveMapCanvas');
  if (!canvas) return;
  if (!isLiveMapInitialized) {
      setupLiveCanvasEvents();
      isLiveMapInitialized = true;
      console.log("Live Map Initialized (Once)");
  }
}

// ฟังก์ชันสำหรับติดตั้ง Event Listeners
function setupLiveCanvasEvents() {
  canvas.addEventListener('mousedown', (e) => mapView.handleMouseDown(e));
  canvas.addEventListener('mousemove', (e) => mapView.handleMouseMove(e));
  canvas.addEventListener('mouseup', (e) => mapView.handleMouseUp(e));
  canvas.addEventListener('mouseleave', (e) => mapView.handleMouseUp(e));
}

export function drawLiveMap() {
    if (!canvas) return;
    const displayWidth = canvas.clientWidth;
    const displayHeight = canvas.clientHeight;
    if (canvas.width !== displayWidth || canvas.height !== displayHeight) {
        canvas.width = displayWidth;
        canvas.height = displayHeight;
        
        // ถ้ามีการปรับขนาด อาจจะอยากให้ Reset View ใหม่อีกรอบ (Optional)
        resetLiveMapView(); 
    }

    const ctx = canvas.getContext('2d');
    
    // ปิด Smoothing เพื่อให้แผนที่คมชัดแบบ Pixel Art
    ctx.imageSmoothingEnabled = false; 

    if (!isLiveMapReady) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = '#666';
        ctx.font = '14px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('Waiting for SLAM data...', canvas.width / 2, canvas.height / 2);
        return; 
    }
    
    // เริ่มวาดแผนที่
    applyTransform(ctx);
    
    // หมุนแผนที่ 90 องศาถ้าจำเป็น (ขึ้นอยู่กับการตั้งค่า TF) 
    // ถ้าแผนที่กลับหัว ให้ลองแก้บรรทัดนี้ หรือเอาออก
    ctx.translate(0, canvas.height);
    ctx.rotate(-Math.PI / 2);
    ctx.scale(1, -1);
    
    ctx.drawImage(offscreenCanvas, 0, 0);

    // วาดหุ่นยนต์ทับ
    drawRobotOnLiveMap(ctx);
    
    restoreTransform(ctx);
}


export function resetLiveMapView() {
  if (!canvas || offscreenCanvas.width === 0) return;

  console.log("LiveMap: View reset to fit and center.");
  const zoomX = canvas.width / offscreenCanvas.width;
  const zoomY = canvas.height / offscreenCanvas.height;

  const newScale = Math.min(zoomX, zoomY) * 0.95; // ย่อให้มีขอบเล็กน้อย

  mapView.viewState.scale = newScale;
  mapView.viewState.offsetX = (canvas.width - offscreenCanvas.width * newScale) / 2;
  mapView.viewState.offsetY = (canvas.height - offscreenCanvas.height * newScale) / 2;
  
  drawLiveMap(); // วาดใหม่ด้วยค่าที่คำนวณได้
}