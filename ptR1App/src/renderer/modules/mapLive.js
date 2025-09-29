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
/**
 * ประมวลผลและเก็บข้อมูลแผนที่ลงบน Off-screen canvas
 * @param {object} mapData - The OccupancyGrid message from ROS.
 */


export function processLiveMapData(mapData) {
    if (!mapData || !mapData.info || !mapData.data) return;

    currentMapInfo = mapData.info;

    // ใช้ .some() เพื่อหาว่ามี pixel ไหนใน array ที่ไม่ใช่ -1 (Unknown) หรือไม่
    const hasMeaningfulData = mapData.data.some(value => value !== -1);
    
    // ถ้าเจอข้อมูลที่ใช้งานได้จริง ถึงจะตั้งค่า Flag
    if (hasMeaningfulData) {
        isLiveMapReady = true;
    }

    const width = mapData.info.width;
    const height = mapData.info.height;

    if (offscreenCanvas.width !== width || offscreenCanvas.height !== height) {
        offscreenCanvas.width = width;
        offscreenCanvas.height = height;
    }

    const imageData = offscreenCtx.createImageData(width, height);
    const data = imageData.data;

    for (let i = 0; i < mapData.data.length; i++) {
        const occupancyValue = mapData.data[i];
        const pixelIndex = i * 4;
        if (occupancyValue === -1) {
            data[pixelIndex] = 128; data[pixelIndex + 1] = 128; data[pixelIndex + 2] = 128;
        } else if (occupancyValue === 0) {
            data[pixelIndex] = 255; data[pixelIndex + 1] = 255; data[pixelIndex + 2] = 255;
        } else {
            data[pixelIndex] = 0; data[pixelIndex + 1] = 0; data[pixelIndex + 2] = 0;
        }
        data[pixelIndex + 3] = 255;
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
  //console.log("Drawing robot at position:", latestRobotPose.position);
  
  const px = (pose.position.x - origin.position.x) / resolution;
  const py = height - ((pose.position.y - origin.position.y) / resolution);
  //console.log(`Calculated px: ${px}, py: ${py}`);

  //console.log("POSE:", JSON.stringify(latestRobotPose.position, null, 2));
  //console.log("MAP INFO:", JSON.stringify(currentMapInfo, null, 2));

  ctx.beginPath();
  ctx.arc(px, py, 5, 0, 2 * Math.PI, false);
  ctx.fillStyle = 'rgba(0, 150, 255, 0.8)';
  ctx.fill();
  ctx.lineWidth = 1;
  ctx.strokeStyle = '#FFFFFF';
  ctx.stroke();

  const yaw = quaternionToYaw(pose.orientation);
  const arrowLength = 10;
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
  setupLiveCanvasEvents();
}

// ฟังก์ชันสำหรับติดตั้ง Event Listeners
function setupLiveCanvasEvents() {
  canvas.addEventListener('mousedown', (e) => {
    // Live map ทำแค่ Pan อย่างเดียวเสมอ
    mapView.handleMouseDown(e);
  });

  canvas.addEventListener('mousemove', (e) => {
    mapView.handleMouseMove(e);
  });

  canvas.addEventListener('mouseup', (e) => {
    mapView.handleMouseUp(e);
  });

  canvas.addEventListener('mouseleave', (e) => {
    mapView.handleMouseUp(e);
  });
}

/**
 * วาดแผนที่จาก Off-screen canvas ลงบน Canvas จริง
 * @param {HTMLCanvasElement} visibleCanvas - The canvas element to draw on.
 */
export function drawLiveMap() {
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    ctx.imageSmoothingEnabled = false;
    if (!isLiveMapReady) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = 'gray';
        ctx.font = '12px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('Waiting for SLAM or map data...', canvas.width / 2, canvas.height / 2);
        return; 
    }
    applyTransform(ctx);
    ctx.translate(0, canvas.height);
    ctx.rotate(-Math.PI / 2)
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