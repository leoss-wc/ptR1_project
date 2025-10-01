//resume/stop/start, index estimate

// modules/patrol.js
// 👉 จัดการการวาด path, การส่ง patrol, การหยุด และ resume;
import * as patrolState from './patrolState.js';
import { renderDashboardMap } from './mapHome.js';



//คำนวณพิกัดจาก pixel → world
//ฟังก์ชัน helper สำหรับแปลงพิกัด
function pixelToWorld(pixelX, pixelY, canvasWidth, canvasHeight, mapMeta) {
    // การคำนวณนี้ต้องสอดคล้องกับวิธีการแสดงผลแผนที่ของคุณ
    // สมมติว่าจุด (0,0) ของแผนที่อยู่ตรงกลาง canvas
    const xPixelFromCenter = pixelX - canvasWidth / 2;
    const yPixelFromCenter = pixelY - canvasHeight / 2;

    const { resolution, origin, height: mapHeightInPixels } = mapMeta;

    // การแปลงแกน Y จะกลับด้าน (ในระบบภาพ Y ชี้ลง, ในระบบ ROS Y ชี้ขึ้น)
    // การคำนวณนี้อาจต้องปรับตาม YAML และวิธีการแสดงผลของคุณ
    const xWorld = origin[0] + (xPixelFromCenter * resolution);
    const yWorld = origin[1] - (yPixelFromCenter * resolution);

    return { x: xWorld, y: yWorld };
}



// 🎯 กำหนด goal เดี่ยว (คลิกที่ canvas)
export function enableSingleGoal(canvas, activeMap) {
  canvas.addEventListener('contextmenu', (e) => {
    e.preventDefault();
    if (!activeMap || !activeMap.meta) {
        console.warn("Cannot set single goal: Active map or meta is missing.");
        return;
    }

    const rect = canvas.getBoundingClientRect();
    // คำนวณพิกัดบนแผนที่จริง
    const worldCoords = pixelToWorld(
      e.clientX - rect.left,
      e.clientY - rect.top,
      canvas.width,
      canvas.height,
      activeMap.meta
    );

    console.log("Sending single goal:", worldCoords);
    window.electronAPI.sendSingleGoal(worldCoords);
    patrolState.stopPatrolState(); // หยุด patrol หากมีการส่ง single goal
  });
}
// 🟢 เริ่มต้นการลาดตระเวน
export function startPatrol() {
  patrolState.startPatrolState();
  if (patrolState.isPatrolling) {
    console.log("Sending start patrol command with goal:", patrolState.goalPoint);
    window.electronAPI.sendSingleGoal(patrolState.goalPoint);
  }
}

// ⏸️ หยุดการลาดตระเวนชั่วคราว
export function pausePatrol() {
    patrolState.pausePatrolState();
    console.log("Sending stop (cancel goal) command");
    
    window.electronAPI.cancelCurrentGoal();
}

// ▶️ กลับมาลาดตระเวนต่อ
export function resumePatrol() {
    patrolState.resumePatrolState();
    if (patrolState.isPatrolling) {
        console.log("Sending resume patrol command with goal:", patrolState.goalPoint);
        window.electronAPI.sendSingleGoal(patrolState.goalPoint);
    }
}

// 🔴 หยุดและรีเซ็ตการลาดตระเวน
export function stopPatrol() {
  patrolState.stopPatrolState();
  console.log("Sending stop (cancel goal) command");
  window.electronAPI.cancelCurrentGoal(); // สมมติว่ามี API นี้เพื่อหยุดหุ่นยนต์
}

export function initPatrolManager() {
  console.log("Patrol Manager Initialized.");
  // ฟังสถานะการเคลื่อนที่จาก main process
  window.electronAPI.onGoalResult(handleGoalResult);
}

function handleGoalResult(result) {
  console.log('Patrol Manager received goal result:', result);

  switch (result.status) {
    case 'SUCCEEDED':
      console.log("Goal reached successfully. Moving to next...");
      const nextGoal = patrolState.moveToNextGoal();
      if (nextGoal) {
          window.electronAPI.sendSingleGoal(nextGoal);
      } else {
          console.log("Patrol finished!");
      }
      break;

    case 'ABORTED':
    case 'REJECTED':
    case 'LOST':
      patrolState.stopPatrolState(); // หยุดการลาดตระเวน
      alert(`Patrol Failed! Reason: ${result.status}\nDetails: ${result.text}`);
      break;
    
    case 'PREEMPTED':
    case 'RECALLED':
       // ผู้ใช้สั่งหยุดหรือส่ง Goal ใหม่เอง ไม่ต้องทำอะไรเป็นพิเศษ
       console.log(`Patrol stopped by user action (${result.status}).`);
       break;
  }
  
  // สั่งวาดใหม่เพื่ออัปเดต UI
  if (typeof renderDashboardMap === 'function') {
      renderDashboardMap();
  }
}
