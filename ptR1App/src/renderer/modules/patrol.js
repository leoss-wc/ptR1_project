//resume/stop/start, index estimate

// modules/patrol.js
// จัดการการวาด path, การส่ง patrol, การหยุด และ resume;
import * as patrolState from './patrolState.js';
import { renderDashboardMap } from './mapHome.js';
import { robotPose,  } from './robotState.js';

// เริ่มต้นการลาดตระเวน
export function startPatrol() {
  patrolState.startPatrolState();
  if (patrolState.isPatrolling) {
    console.log("Sending start patrol command with goal:", patrolState.goalPoint);
    window.electronAPI.sendSingleGoal({ pose: patrolState.goalPoint });
  }
}

// หยุดการลาดตระเวนชั่วคราว
export function pausePatrol() {
    patrolState.pausePatrolState();
    console.log("Sending stop (cancel goal) command");
    
    window.electronAPI.cancelCurrentGoal();
}

// กลับมาลาดตระเวนต่อ
export function resumePatrol() {
    patrolState.resumePatrolState();
    if (patrolState.isPatrolling) {
        console.log("Sending resume patrol command with goal:", patrolState.goalPoint);
        window.electronAPI.sendSingleGoal({ pose: patrolState.goalPoint });
    }
}

// หยุดและรีเซ็ตการลาดตระเวน
export function stopPatrol() {
  patrolState.stopPatrolState();
  console.log("Sending stop (cancel goal) command");
  window.electronAPI.cancelCurrentGoal();
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
          window.electronAPI.sendSingleGoal({ pose: patrolState.goalPoint });
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
