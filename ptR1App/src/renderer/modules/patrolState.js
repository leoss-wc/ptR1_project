// patrolState.js
// เก็บสถานะของการลาดตระเวน เช่น path ทั้งหมด และ goal point ปัจจุบัน

// 📍 path ทั้งหมดที่หุ่นยนต์ต้องเดิน (จาก Electron หรือ Planner)
export let patrolPath = [];
export let goalPoint = null; // goal point ปัจจุบัน
export let isPatrolling = false; // สถานะว่ากำลังลาดตระเวน
export let currentGoalIndex = -1; // ดัชนีของเป้าหมายปัจจุบัน
export let isLooping = false; // สถานะการวน patrol


// อัปเดต path ลาดตระเวน
export function setPatrolPath(path) {
  patrolPath = [...path];
}

// ตั้ง goal ใหม่
export function setGoalPoint(pt) {
  goalPoint = pt;
}

// ล้าง goal point
export function clearGoalPoint() {
  goalPoint = null;
}

export function setLooping(value) {
  isLooping = value;
  console.log(`Patrol looping mode set to: ${isLooping}`);
}


// ฟังก์ชันสำหรับควบคุมสถานะการ Patrol
export function startPatrolState() {
  if (patrolPath.length === 0) {
    console.warn("No patrol path set. Cannot start patrol.");
    return;
  }
  isPatrolling = true;
  currentGoalIndex = 0;
  setGoalPoint(patrolPath[currentGoalIndex]);
  console.log("Patrol started. Current goal index:", currentGoalIndex);
}

// ⏸️ หยุดการลาดตระเวนชั่วคราว
export function pausePatrolState() {
  isPatrolling = false;
  console.log("Patrol paused.");
}

// ▶️ กลับมาลาดตระเวนต่อ
export function resumePatrolState() {
  if (currentGoalIndex === -1 || patrolPath.length === 0) {
    console.warn("Cannot resume patrol. No current goal or patrol path set.");
    return;
  }
  isPatrolling = true;
  console.log("Patrol resumed.");
  // ส่ง goal ปัจจุบันอีกครั้งเพื่อเริ่มทำงาน
  setGoalPoint(patrolPath[currentGoalIndex]);
}

// หยุดการลาดตระเวนและรีเซ็ตสถานะ
export function stopPatrolState() {
  isPatrolling = false;
  currentGoalIndex = -1;
  clearGoalPoint();
  console.log("Patrol stopped and reset.");
}

// จัดการไปยังเป้าหมายถัดไปใน path
export function moveToNextGoal() {
  // --- กรณีที่ยังไม่ถึงจุดสุดท้าย ---
  if (currentGoalIndex < patrolPath.length - 1) {
    currentGoalIndex++;
    setGoalPoint(patrolPath[currentGoalIndex]);
    console.log(`Moving to next goal (index ${currentGoalIndex}).`);
    return goalPoint;
  } 
  
  // --- กรณีที่ถึงจุดสุดท้ายแล้ว ---
  if (isLooping) {
    currentGoalIndex = 0;
    setGoalPoint(patrolPath[currentGoalIndex]);
    console.log("Looping back to the first goal (index 0).");
    return goalPoint;
  } else {
    // ถ้าปิดโหมด Loop: หยุดการทำงาน
    stopPatrolState(); 
    return null;
  }
}