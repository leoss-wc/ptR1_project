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
    const statusLower = newStatus.toLowerCase();
    if (statusLower === 'active' || statusLower === 'patrolling' || statusLower === 'paused') {
        // กรณี Active หรือ Paused ถือว่ายังอยู่ใน "โหมดตรวจการณ์" (เส้นทางควรยังแสดงอยู่)
        isPatrolling = true;
    } else {
        // กรณี Idle, Stopped, Finished หรืออื่นๆ ให้ปิดโหมด
        isPatrolling = false;
    }

    console.log(`State Updated: Status="${currentStatus}", isPatrolling=${isPatrolling}`);
    // แจ้งเตือน Listener
    statusListeners.forEach(cb => cb(currentStatus));
}

export function setLooping(value) {
  isLooping = value;
  console.log(`Patrol looping mode set to: ${isLooping}`);
}

export function clearDrawnPath() {
    patrolPath.length = 0; 
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
