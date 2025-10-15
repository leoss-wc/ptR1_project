// ./modules/patrolState.js

export let patrolPath = [];
export let isLooping = false;
export let currentStatus = "Idle";
export let goalPoint = null;
export let isPatrolling = false;

let statusListeners = [];

// ฟังก์ชันสำหรับให้ส่วนอื่นของ UI มาติดตามสถานะ
export function addStatusListener(callback) {
    statusListeners.push(callback);
}

// ฟังก์ชันสำหรับอัปเดตสถานะและแจ้งเตือน Listener ทั้งหมด
export function updateStatus(newStatus) {
    currentStatus = newStatus;
    statusListeners.forEach(cb => cb(currentStatus));
}

export function setLooping(value) {
  isLooping = value;
  console.log(`Patrol looping mode set to: ${isLooping}`);
}
// เพิ่มฟังก์ชันนี้เข้าไป
export function clearDrawnPath() {
    patrolPath.length = 0; // วิธีเคลียร์ array ที่เร็วและปลอดภัย
    updateStatus("Idle"); // อาจจะรีเซ็ตสถานะด้วย
}

export function setGoalPoint(pose) {
    goalPoint = pose;
}

export function clearGoalPoint() {
    goalPoint = null;
}

export function setPatrolling(status) {
    isPatrolling = status;
}
