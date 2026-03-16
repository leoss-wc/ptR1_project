// ./modules/patrol.js

import * as patrolState from './patrolState.js';
import { patrolPath as drawnPath, clearDrawnPath } from './patrolState.js';

export function initPatrolManager() {
    // 1. Listeners รับผลลัพธ์การสั่งงาน (Service Result)
    window.electronAPI.onPatrolStartResult(handleServiceResult);
    window.electronAPI.onPatrolPauseResult(handleServiceResult);
    window.electronAPI.onPatrolResumeResult(handleServiceResult);
    window.electronAPI.onPatrolStopResult(handleServiceResult);

    // 2. Listener รับสถานะการลาดตระเวน (Status Change)
    window.electronAPI.onPatrolStatusChange((status) => {
        console.log("Patrol Status Changed:", status);
        patrolState.updateStatus(status); // ส่งต่อไปให้ State Manager จัดการ
    });
    
    // Listener รอรับผลลัพธ์ของ Goal
    window.electronAPI.onGoalResult(result => {
        if (result.status === 'SUCCEEDED') {
            patrolState.updateStatus('Goal Reached');
        } else if (result.status !== 'PREEMPTED' && result.status !== 'RECALLED') {
            patrolState.updateStatus(`Goal Failed! Status: ${result.status}`);
        }
    });

    // อัปเดต Label บนหน้าเว็บเมื่อ State เปลี่ยน
    const statusLabel = document.getElementById('patrol-status-label');
    patrolState.addStatusListener(newStatus => {
        if (statusLabel) {
            statusLabel.textContent = newStatus;
        }
    });
    
    console.log("New Patrol Manager Initialized.");
}

function handleServiceResult(result) {
    console.log("Patrol Service Result:", result);
    // ถ้าสั่งงานไม่สำเร็จ ให้แจ้งเตือน แต่ถ้าสำเร็จให้รอ status update จาก ROS
    if (!result.success) {
        patrolState.updateStatus(`Error: ${result.message}`);
    } else {
        patrolState.updateStatus(result.message);
    }
}

export function startPatrol() {
    if (drawnPath.length < 1) {
        alert("Please draw a path or set at least one goal.");
        return;
    }
    const shouldLoop = patrolState.isLooping;
    const goals = drawnPath.map(point => ({
        header: { 
            frame_id: 'map',  // ระบุว่าพิกัดนี้อยู่บนแผนที่
            stamp: { secs: 0, nsecs: 0 } 
        },
        pose: {
            position: { x: point.x, y: point.y, z: 0.0 },
            orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
        }
    }));

    patrolState.setPatrolling(true);
    patrolState.updateStatus(`Starting patrol with ${goals.length} points...`);
    
    // ส่งข้อมูลที่ถูกต้องไป
    window.electronAPI.startPatrol(goals, shouldLoop);
}

export function pausePatrol() {
    window.electronAPI.pausePatrol();
}

export function resumePatrol() {
    window.electronAPI.resumePatrol();
}

export function stopPatrol() {
    window.electronAPI.stopPatrol();
}

export function saveDrawnPath() {
    alert("Save path feature not implemented yet.");
}