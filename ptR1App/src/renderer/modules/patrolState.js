// patrolState.js
export let patrolPath = [];
export let goalPoint = null;
export let isPatrolling = false;
export let currentGoalIndex = -1;
export let isLooping = false;

// ✨ เพิ่ม: ฟังก์ชันสำหรับแปลงมุม Yaw เป็น Quaternion
function yawToQuaternion(yaw) {
  const halfYaw = yaw / 2.0;
  return {
    x: 0,
    y: 0,
    z: Math.sin(halfYaw),
    w: Math.cos(halfYaw),
  };
}

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

// 🔧 แก้ไข: คำนวณทิศทางสำหรับ Goal ถัดๆ ไป
export function moveToNextGoal() {
  const previousIndex = currentGoalIndex;

  // --- หา Index ของ Goal ถัดไป ---
  let nextIndex;
  if (currentGoalIndex < patrolPath.length - 1) {
    nextIndex = currentGoalIndex + 1;
  } else if (isLooping) {
    nextIndex = 0;
  } else {
    stopPatrolState();
    return null;
  }
  currentGoalIndex = nextIndex;
  
  // --- คำนวณทิศทาง ---
  const currentPosition = patrolPath[previousIndex];
  const nextPosition = patrolPath[nextIndex];
  let orientation = { x: 0, y: 0, z: 0, w: 1 };

  // คำนวณทิศทางจากจุด "ปัจจุบัน" ไปยังจุด "ถัดไป"
  const dx = nextPosition.x - currentPosition.x;
  const dy = nextPosition.y - currentPosition.y;

  if (Math.hypot(dx, dy) > 0.01) { // ป้องกันการคำนวณจากจุดเดียวกัน
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