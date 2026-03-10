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
let previousRawData = null;

// เตรียมค่าสีไว้ล่วงหน้า (ABGR Format สำหรับ Little Endian Systems)
// 0xFF = 255
const COLOR_UNKNOWN = 0xFF808080; // Gray (128, 128, 128, 255)
const COLOR_FREE = 0xFFFFFFFF;    // White (255, 255, 255, 255)
const COLOR_OCCUPIED = 0xFF000000; // Black (0, 0, 0, 255)


export function processLiveMapData(mapData) {
    if (!mapData || !mapData.info || !mapData.data) return;

    currentMapInfo = mapData.info;
    const rawData = mapData.data; 

    const width = mapData.info.width;
    const height = mapData.info.height;

    // เช็คว่าต้องรีเซ็ตแผนที่ใหม่ทั้งหมดไหม (เช่น เปิดครั้งแรก หรือขนาดแผนที่เปลี่ยน)
    let isFullUpdate = !previousRawData || 
                       previousRawData.length !== rawData.length || 
                       offscreenCanvas.width !== width || 
                       offscreenCanvas.height !== height;

    if (isFullUpdate) {
        offscreenCanvas.width = width;
        offscreenCanvas.height = height;
        isLiveMapReady = rawData.some(value => value !== -1);
    }

    const imageData = offscreenCtx.createImageData(width, height);
    const buf32 = new Uint32Array(imageData.data.buffer);

    // ตัวแปรสำหรับหากรอบ "Dirty Bounding Box"
    let minX = width, minY = height, maxX = -1, maxY = -1;
    let hasChanges = false;

    for (let y = 0; y < height; y++) {
        const rosRowStart = y * width;
        const canvasRowStart = (height - 1 - y) * width;
        const canvasY = height - 1 - y; // แกน Y ของ Canvas

        for (let x = 0; x < width; x++) {
            const idx = rosRowStart + x;
            const val = rawData[idx];

            // เช็คว่าจุดนี้เปลี่ยนไปจากเดิมไหม หรือเป็นการบังคับวาดใหม่ทั้งหมด
            if (isFullUpdate || val !== previousRawData[idx]) {
                hasChanges = true;
                
                // ลงสีใหม่เฉพาะจุดที่เปลี่ยน
                buf32[canvasRowStart + x] = val === -1 ? COLOR_UNKNOWN : (val === 0 ? COLOR_FREE : COLOR_OCCUPIED);

                // ขยายกรอบ Bounding Box ให้ครอบคลุมจุดนี้
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (canvasY < minY) minY = canvasY;
                if (canvasY > maxY) maxY = canvasY;
            } else if (!isFullUpdate) {
                // ถ้าไม่เปลี่ยน และไม่ใช่ Full Update ให้ดึงสีเดิมมาใส่ไว้ด้วย
                // (เพราะ createImageData สร้างพื้นใสเปล่าๆ ขึ้นมาใหม่)
                buf32[canvasRowStart + x] = val === -1 ? COLOR_UNKNOWN : (val === 0 ? COLOR_FREE : COLOR_OCCUPIED);
            }
        }
    }

    // เก็บข้อมูลรอบนี้ไว้เป็น "ข้อมูลเก่า" สำหรับเทียบในรอบหน้า
    previousRawData = rawData; 

    // วาดลง Canvas เฉพาะส่วนที่มีการเปลี่ยนแปลง
    if (hasChanges) {
        if (isFullUpdate) {
            // วาดใหม่ทั้งแผ่น
            offscreenCtx.putImageData(imageData, 0, 0);
        } else {
            // ส่งไปให้ GPU เฉพาะ "กรอบสี่เหลี่ยม" ที่มีการเปลี่ยนแปลง
            const dirtyWidth = maxX - minX + 1;
            const dirtyHeight = maxY - minY + 1;
            
            // putImageData(imageData, dx, dy, dirtyX, dirtyY, dirtyWidth, dirtyHeight)
            offscreenCtx.putImageData(imageData, 0, 0, minX, minY, dirtyWidth, dirtyHeight);
            
            // console.log(`Updated region: X:${minX} Y:${minY} W:${dirtyWidth} H:${dirtyHeight}`);
        }
    }
}


function quaternionToYaw(q) {
  return Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

function drawRobotOnLiveMap(ctx) {
    if (!latestRobotPose || !currentMapInfo) return;

    const { resolution, origin, height } = currentMapInfo;
    const pose = latestRobotPose;
  
    // คำนวณพิกัด Pixel (หักลบด้วย Height เพื่อให้ตรงกับแผนที่ที่ถูกพลิกแกน Y แล้ว)
    const px = (pose.position.x - origin.position.x) / resolution;
    const py = height - ((pose.position.y - origin.position.y) / resolution);

    const yaw = quaternionToYaw(pose.orientation);

    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-yaw);

    // วาดตัวหุ่นยนต์ (วงกลม)
    ctx.beginPath();
    ctx.arc(0, 0, 5, 0, 2 * Math.PI, false);
    ctx.fillStyle = 'rgba(0, 150, 255, 0.8)';
    ctx.fill();
    ctx.lineWidth = 1;
    ctx.strokeStyle = '#FFFFFF';
    ctx.stroke();

    // วาดลูกศรบอกทิศทาง (ชี้ไปทางแกน +X แบบ ROS)
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(15, 0);
    ctx.strokeStyle = '#FFFFFF';
    ctx.lineWidth = 2;
    ctx.stroke();

    ctx.restore();
}
export function updateLiveRobotPose(pose) {
  latestRobotPose = pose;
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
  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    e.stopPropagation();
    mapView.handleWheel(e);
  }, { passive: false });
}

export function drawLiveMap() {
    if (!canvas) return;
    const displayWidth = canvas.clientWidth;
    const displayHeight = canvas.clientHeight;
    if (canvas.width !== displayWidth || canvas.height !== displayHeight) {
        canvas.width = displayWidth;
        canvas.height = displayHeight;
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
    ctx.drawImage(offscreenCanvas, 0, 0);
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