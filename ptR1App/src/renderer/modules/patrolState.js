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
export function updateStatus(newStatus, forcePatrollingState = null) {
    currentStatus = newStatus;
    const statusLower = newStatus.toLowerCase();
    if (forcePatrollingState !== null) {
        isPatrolling = forcePatrollingState;
    } 
    else {
        if (statusLower.includes('patrolling') || 
            statusLower.includes('active') || 
            statusLower === 'paused' ||
            statusLower.includes('goal reached') ||
            statusLower.includes('moving')) {       
            isPatrolling = true;
        } else if (statusLower.includes('idle') || 
                   statusLower.includes('finished') || 
                   statusLower.includes('stopped')) {
            isPatrolling = false;
        }
        // กรณีอื่นๆ (เช่น Error) ให้คงค่าเดิมไว้ ไม่ต้องไปเปลี่ยนมัน
    }

    console.log(`State Updated: Status="${currentStatus}", isPatrolling=${isPatrolling}`);
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
