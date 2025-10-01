import { yawToQuaternion } from './utils.js';
// patrolState.js
export let patrolPath = [];
export let goalPoint = null;
export let isPatrolling = false;
export let currentGoalIndex = -1;
export let isLooping = false;


export function setPatrolPath(path) {
  patrolPath = [...path];
}

export function setGoalPoint(pose) {
  goalPoint = pose;
}

export function clearGoalPoint() {
  goalPoint = null;
}

export function setLooping(value) {
  isLooping = value;
  console.log(`Patrol looping mode set to: ${isLooping}`);
}

// 🔧 แก้ไข: คำนวณทิศทางสำหรับ Goal แรก
export function startPatrolState() {
  if (patrolPath.length === 0) {
    console.warn("No patrol path set. Cannot start patrol.");
    return;
  }
  isPatrolling = true;
  currentGoalIndex = 0;

  const firstPosition = patrolPath[0];
  let orientation = { x: 0, y: 0, z: 0, w: 1 }; // ทิศทางเริ่มต้น

  // ถ้ามีจุดถัดไป (จุดที่ 2) ให้คำนวณทิศทางจากจุดแรกไปหาจุดที่สอง
  if (patrolPath.length > 1) {
    const secondPosition = patrolPath[1];
    const dx = secondPosition.x - firstPosition.x;
    const dy = secondPosition.y - firstPosition.y;
    const yaw = Math.atan2(dy, dx);
    orientation = yawToQuaternion(yaw);
  }

  const firstPose = {
    position: firstPosition,
    orientation: orientation
  };
  setGoalPoint(firstPose);
  console.log("Patrol started. Current goal index:", currentGoalIndex);
}

export function pausePatrolState() {
  isPatrolling = false;
  console.log("Patrol paused.");
}

export function resumePatrolState() {
  if (currentGoalIndex === -1 || patrolPath.length === 0) {
    console.warn("Cannot resume patrol. No current goal or patrol path set.");
    return;
  }
  isPatrolling = true;
  console.log("Patrol resumed.");
  setGoalPoint(goalPoint); // ใช้ goalPoint เดิมที่ถูก pause ไว้
}

export function stopPatrolState() {
  isPatrolling = false;
  currentGoalIndex = -1;
  clearGoalPoint();
  console.log("Patrol stopped and reset.");
}

export function moveToNextGoal() {
  const previousIndex = currentGoalIndex;

  let nextIndex;
  // --- กรณีถึงจุดสุดท้าย ---
  if (currentGoalIndex >= patrolPath.length - 1) {
    if (isLooping) {
        //  START: Logic ใหม่สำหรับจัดการ Loop
        const lastIndex = patrolPath.length - 1;
        // ตรวจสอบว่ามี Path มากกว่า 1 จุด และจุดแรกกับจุดสุดท้ายเป็นตำแหน่งเดียวกันหรือไม่
        if (patrolPath.length > 1 &&
            patrolPath[0].x === patrolPath[lastIndex].x &&
            patrolPath[0].y === patrolPath[lastIndex].y) 
        {
            // ถ้าใช่ ให้ข้ามไปที่จุดที่ 2 (index 1) เลย
            nextIndex = 1;
            console.log("Closed loop detected. Skipping to index 1.");
        } else {
            // ถ้าไม่ใช่ Loop แบบปิด ให้กลับไปที่จุดแรกตามปกติ
            nextIndex = 0;
            console.log("Looping back to the first goal (index 0).");
        }
        // ✨ END: Logic ใหม่
    } else {
      // ถ้าไม่วน Loop ให้หยุด
      stopPatrolState();
      return null;
    }
  } 
  // --- กรณียังไม่ถึงจุดสุดท้าย ---
  else {
    nextIndex = currentGoalIndex + 1;
  }
  
  currentGoalIndex = nextIndex;

  const currentPosition = patrolPath[previousIndex];
  const nextPosition = patrolPath[nextIndex];
  let orientation = { x: 0, y: 0, z: 0, w: 1 };

  const dx = nextPosition.x - currentPosition.x;
  const dy = nextPosition.y - currentPosition.y;

  if (Math.hypot(dx, dy) > 0.01) {
      const yaw = Math.atan2(dy, dx);
      orientation = yawToQuaternion(yaw);
  }
  
  const nextPose = {
    position: nextPosition,
    orientation: orientation
  };

  setGoalPoint(nextPose);
  console.log(`Moving to next goal (index ${currentGoalIndex}) with calculated orientation.`);
  return goalPoint;
}