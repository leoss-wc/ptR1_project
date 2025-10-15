// ./modules/patrol.js (เวอร์ชันใหม่)

import * as patrolState from './patrolState.js';
import { patrolPath as drawnPath, clearDrawnPath } from './patrolState.js'; // ใช้ path จาก state โดยตรง

export function initPatrolManager() {
    // ตั้งค่า Listener รอรับผลลัพธ์จาก Service ต่างๆ ของ Patrol
    window.electronAPI.onPatrolStartResult(handleServiceResult);
    window.electronAPI.onPatrolPauseResult(handleServiceResult);
    window.electronAPI.onPatrolResumeResult(handleServiceResult);
    window.electronAPI.onPatrolStopResult(handleServiceResult);
    
    // Listener รอรับผลลัพธ์สุดท้ายของ Goal แต่ละจุด (ยังคงมีประโยชน์ในการแสดงสถานะ)
    window.electronAPI.onGoalResult(result => {
        if (result.status === 'SUCCEEDED') {
            patrolState.updateStatus('✅ Goal Reached! Moving to next...');
        } else if (result.status !== 'PREEMPTED' && result.status !== 'RECALLED') {
            // ถ้าล้มเหลว (ที่ไม่ใช่การสั่งหยุดโดยผู้ใช้) ให้แสดงสถานะ
            patrolState.updateStatus(`❌ Goal Failed! Status: ${result.status}`);
        }
    });

    const statusLabel = document.getElementById('patrol-status-label');
    patrolState.addStatusListener(newStatus => {
        if (statusLabel) {
            statusLabel.textContent = newStatus;
        }
    });
    
    console.log("New Patrol Manager Initialized (Service-based).");
}

function handleServiceResult(result) {
    console.log("Patrol Service Result:", result);
    patrolState.updateStatus(result.message); // แสดงข้อความจาก Backend
}

export function startPatrol() {
    if (drawnPath.length < 1) {
        alert("Please draw a path or set at least one goal.");
        return;
    }
    const shouldLoop = patrolState.isLooping;


    // แปลง path (array of {x, y}) เป็น geometry_msgs/PoseStamped[]
    // นี่คือส่วนสำคัญที่สร้าง "ภารกิจ" ทั้งหมด
    const goals = drawnPath.map(point => ({
        header: { frame_id: 'map' },
        pose: {
            position: { x: point.x, y: point.y, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 } // Backend จะคำนวณทิศทางให้เอง
        }
    }));
    patrolState.setPatrolling(true);
    patrolState.updateStatus(`Starting patrol with ${goals.length} points...`);
    console.log("Starting patrol with goals:", goals, "Looping:", shouldLoop);
    window.electronAPI.startPatrol(goals, shouldLoop);
}

export function pausePatrol() {
    patrolState.updateStatus("⏸️ Pausing patrol...");
    window.electronAPI.pausePatrol();
}

export function resumePatrol() {
    patrolState.updateStatus("▶️ Resuming patrol...");
    window.electronAPI.resumePatrol();
}

export function stopPatrol() {
    patrolState.updateStatus("⏹️ Stopping patrol...");
    patrolState.setPatrolling(false);
    window.electronAPI.stopPatrol();
    clearDrawnPath(); // อาจจะเคลียร์เส้นที่วาดไว้ด้วยเมื่อสั่งหยุด
}

export function saveDrawnPath() {
    alert("Save path feature not implemented yet.");
}


