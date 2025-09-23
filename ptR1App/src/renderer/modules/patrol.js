//resume/stop/start, index estimate

// modules/patrol.js
// 👉 จัดการการวาด path, การส่ง patrol, การหยุด และ resume
import { patrolPath } from './patrolState.js';
import { activeMap } from './mapState.js';
import { goalPoint } from './patrolState.js';

// 🔁 เก็บ path ที่ลาก
const patrolPath = [];
let isDrawing = false;


// 🖱️ เริ่มจับ path ด้วย mouse บน canvas
export function enablePatrolDrawing(canvas) {
  canvas.addEventListener('mousedown', (e) => {
    if (!activeMap?.meta) return;
    isDrawing = true;
    patrolPath.length = 0;  // clear path เก่า
    addPoint(e, canvas);
    console.log("Start drawing patrol path");
  });

  canvas.addEventListener('mousemove', (e) => {
    if (isDrawing) addPoint(e, canvas);
  });

  canvas.addEventListener('mouseup', () => {
    isDrawing = false;
    window.electronAPI.sendPatrolPath(patrolPath);
  });
}

// 🧠 คำนวณพิกัดจาก pixel → world
function addPoint(e, canvas) {
  const rect = canvas.getBoundingClientRect();
  const xPixel = e.clientX - rect.left - canvas.width / 2;
  const yPixel = e.clientY - rect.top - canvas.height / 2;

  const resolution = activeMap.meta.resolution;
  const origin = activeMap.meta.origin;
  const imgH = activeMap.meta.height;

  const xWorld = (xPixel / resolution) + origin[0];
  const yWorld = ((imgH * resolution - yPixel) / resolution) + origin[1];

  patrolPath.push({ x: xWorld, y: yWorld });
}


// 🎯 กำหนด goal เดี่ยว (คลิกที่ canvas)
export function enableSingleGoal(canvas) {
  canvas.addEventListener('contextmenu', (e) => {
    e.preventDefault();
    const rect = canvas.getBoundingClientRect();
    const xPixel = e.clientX - rect.left - canvas.width / 2;
    const yPixel = e.clientY - rect.top - canvas.height / 2;

    const resolution = activeMap.meta.resolution;
    const origin = activeMap.meta.origin;
    const imgH = activeMap.meta.height;

    const xWorld = (xPixel / resolution) + origin[0];
    const yWorld = ((imgH * resolution - yPixel) / resolution) + origin[1];

    goalPoint.x = xWorld;
    goalPoint.y = yWorld;

    window.electronAPI.sendSingleGoal(goalPoint);
  });
}
