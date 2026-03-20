// modules/shutdownManager.js

// ---- Config ----
const HOME_TIMEOUT_MS     = 600000; // 600s timeout สำหรับ goHome
const LOW_BAT_WARN_PCT    = 20;     // แสดง suggestion ครั้งแรกที่ 20%
const BAT_NOTIFY_COOLDOWN = 5 * 60 * 1000; // แสดง suggestion ซ้ำได้ทุก 5 นาที

// ---- State ----
let _isShuttingDown    = false;
let _cancelRequested   = false; // flag สำหรับยกเลิกกลางทาง goHome
let _countdownTimer    = null;
let _rosConnected      = false;
let _lastBatNotifyTime = 0;     // ป้องกัน notify ถี่เกินไป
let _navRunning        = false; // Navigation กำลังทำงานอยู่หรือไม่

// ---- Reset: ใช้ทุกจุดที่ต้องการยกเลิก ----
function _resetState() {
    _isShuttingDown  = false;
    _cancelRequested = false;
    if (_countdownTimer) {
        clearInterval(_countdownTimer);
        _countdownTimer = null;
    }
    hideOverlay();
    console.log(`[Shutdown] ${new Date().toISOString()} - State reset (cancelled).`);
}

// ---- Battery Helper (ใช้ logic เดียวกับ RobotStatusRenderer) ----
function getBatteryPercent(voltage) {
    let pct = 0;
    if      (voltage >= 13.40) pct = 100;
    else if (voltage >= 13.20) pct = 70 + ((voltage - 13.20) / (13.40 - 13.20) * 30);
    else if (voltage >= 12.90) pct = 30 + ((voltage - 12.90) / (13.20 - 12.90) * 40);
    else if (voltage >= 12.00) pct = ((voltage - 12.00) / (12.90 - 12.00) * 30);
    else pct = 0;
    return Math.floor(pct);
}

// ---- Init ----
export function initShutdownManager(getActiveMap) {
    const btn = document.getElementById('shutdown-btn');
    if (!btn) {
        console.warn('[Shutdown] #shutdown-btn not found');
        return;
    }

    // Track ROS connection state
    if (window.electronAPI?.onConnectionStatus) {
        window.electronAPI.onConnectionStatus((status) => {
            _rosConnected = status.connected;
        });
    }

    // Monitor battery จาก robot status
    if (window.electronAPI?.onRobotStatus) {
        window.electronAPI.onRobotStatus((str) => {
            _handleBatteryStatus(str, getActiveMap);
        });
    }

    btn.addEventListener('click', () => startShutdownFlow(getActiveMap));
}

// ---- Navigation State (เรียกจากภายนอกเพื่ออัปเดตสถานะ) ----
export function setNavRunning(running) {
    _navRunning = running;
    console.log(`[Shutdown] Nav running: ${running}`);
}

// ---- Battery Monitor ----
function _handleBatteryStatus(str, getActiveMap) {
    if (!str || !_rosConnected) return;

    const batMatch = str.match(/Bat:([\d\.]+)V/);
    if (!batMatch) return;

    const voltage = parseFloat(batMatch[1]);
    const percent = getBatteryPercent(voltage);

    // --- Warning: แสดง suggestion เท่านั้น (ไม่บังคับอัตโนมัติ เผื่อเซนเซอร์เสีย) ---
    if (percent <= LOW_BAT_WARN_PCT && !_isShuttingDown) {
        const now = Date.now();
        if (now - _lastBatNotifyTime < BAT_NOTIFY_COOLDOWN) return;
        _lastBatNotifyTime = now;

        console.warn(`[Shutdown] Battery LOW: ${percent}% (${voltage}V). Suggesting go home.`);
        showLowBatterySuggestion(percent, voltage, getActiveMap);
    }
}

// --- Suggestion (กด dismiss ได้) ---
function showLowBatterySuggestion(percent, voltage, getActiveMap) {
    // ถ้ามี overlay อยู่แล้ว (เช่น กำลัง shutdown) ไม่แทรก
    if (document.getElementById('shutdown-overlay')) return;

    const el = document.createElement('div');
    el.id = 'bat-suggestion';
    el.style.cssText = `
        position: fixed; bottom: 24px; right: 24px; z-index: 9998;
        background: #2a2a1a; border: 1px solid #f0ad00;
        border-radius: 12px; padding: 16px 20px;
        color: #fff; max-width: 300px;
        box-shadow: 0 4px 20px rgba(0,0,0,0.5);
        animation: slideIn 0.3s ease;
    `;
    el.innerHTML = `
        <div style="display:flex; align-items:center; gap:10px; margin-bottom:12px;">
            <i class="fa-solid fa-battery-low" style="font-size:22px; color:#f0ad00;"></i>
            <div>
                <div style="font-weight:600; color:#f0ad00;">แบตเตอรี่ใกล้หมด</div>
                <div style="font-size:13px; color:#ccc;">${percent}% (${voltage.toFixed(2)}V)</div>
            </div>
            <button id="bat-dismiss" style="
                margin-left:auto; background:none; border:none;
                color:#aaa; font-size:18px; cursor:pointer; padding:0 4px;
            ">✕</button>
        </div>
        <p style="font-size:13px; margin:0 0 12px; color:#ddd; line-height:1.5;">
            ควรให้หุ่นยนต์กลับ Home เพื่อชาร์จแบตเตอรี่
        </p>
        <div style="display:flex; gap:8px;">
            <button id="bat-go-home" style="
                flex:1; padding:8px; border-radius:8px; border:none; cursor:pointer;
                background:#f0ad00; color:#1a1a00; font-size:13px; font-weight:600;
            ">กลับ Home</button>
            <button id="bat-later" style="
                flex:1; padding:8px; border-radius:8px; border:none; cursor:pointer;
                background:#444; color:#fff; font-size:13px;
            ">ทีหลัง</button>
        </div>
    `;
    document.body.appendChild(el);

    document.getElementById('bat-dismiss').onclick = () => el.remove();
    document.getElementById('bat-later').onclick   = () => el.remove();
    document.getElementById('bat-go-home').onclick = () => {
        el.remove();
        _isShuttingDown = true;
        goHomeAndShutdown(getActiveMap);
    };

    // auto-dismiss หลัง 30 วินาที
    setTimeout(() => el?.remove(), 30000);
}


// ---- Manual Shutdown Flow ----
function startShutdownFlow(getActiveMap) {
    if (_isShuttingDown) {
        console.warn('[Shutdown] Already in shutdown flow.');
        return;
    }
    // ยังไม่ set _isShuttingDown = true — รอให้ user กด confirm ก่อน

    document.getElementById('bat-suggestion')?.remove();

    console.log(`[Shutdown] ${new Date().toISOString()} - User initiated shutdown.`);

    if (!_rosConnected) {
        console.warn('[Shutdown] ROS not connected.');
        return;
    }

    if (!_navRunning) {
        showDialog({
            icon: '🚫',
            title: 'ไม่สามารถ Shutdown ได้',
            message: 'Navigation ไม่ได้ทำงานอยู่\nกรุณาเริ่ม Navigation ก่อนทำการ Shutdown\nเพื่อให้หุ่นยนต์สามารถกลับ Home ได้',
            confirmText: 'รับทราบ',
            cancelText: '',
            dismissText: null,
            onConfirm: () => { _isShuttingDown = false; },
            onCancel:  () => { _isShuttingDown = false; },
        });
        return;
    }

    showDialog({
        icon: '⚠️',
        message: 'ต้องการ Shutdown หุ่นยนต์ใช่หรือไม่?\n\nหุ่นยนต์จะพยายามกลับ Home ก่อน แล้วจึง Power Off',
        confirmText: 'กลับ Home แล้ว Shutdown',
        cancelText: 'Power Off เดี๋ยวนี้',
        dismissText: 'ยกเลิก',  
        onConfirm: () => {
            _isShuttingDown = true;
            goHomeAndShutdown(getActiveMap);
        },
        onCancel: () => {
            _isShuttingDown = true;
            startCountdown(doShutdown);
        },
        onDismiss: () => {
            // ไม่ทำอะไร — user กดผิดหรือเปลี่ยนใจ
            console.log(`[Shutdown] ${new Date().toISOString()} - Dismissed at initial dialog.`);
        },
    });
}

// ---- goHome Logic ----
async function goHomeAndShutdown(getActiveMap) {
    _cancelRequested = false;
    const activeMap = getActiveMap();

    console.log(`[Shutdown] ${new Date().toISOString()} - Going home. Map: ${activeMap?.name ?? 'unknown'}`);

    if (!activeMap?.name) {
        console.warn('[Shutdown] No active map. Skipping goHome.');
        if (!_cancelRequested) startCountdown(doShutdown);
        return;
    }

    // Helper: showStatus พร้อมปุ่มยกเลิกที่ reset state ทั้งหมด
    const showCancellable = (msg) => showStatus(msg, {
        showCancel: true,
        onCancel: () => {
            _cancelRequested = true;
            _resetState();
        },
    });

    // 1. หยุด Patrol ก่อน
    try {
        showCancellable('กำลังหยุด Patrol...');
        await window.electronAPI.stopPatrol();
        await delay(500);
    } catch (err) {
        console.warn('[Shutdown] stopPatrol error (ignored):', err);
    }

    if (_cancelRequested) return;

    // 2. goHome พร้อม Timeout
    showCancellable('กำลังกลับ Home...');
try {
    // ขั้นที่ 1: ส่งคำสั่ง goHome (รอแค่ service ตอบรับ)
    const result = await Promise.race([
        window.electronAPI.goHome(activeMap.name),
        delay(10000).then(() => ({ success: false, message: 'Service Timeout' }))
    ]);

    if (_cancelRequested) return;

    if (!result?.success) {
        console.warn(`[Shutdown] goHome service failed: ${result?.message}`);
        showCancellable(`⚠️ กลับ Home ไม่ได้: ${result?.message}\nกำลัง Shutdown ต่อ...`);
        await delay(2500);
    } else {
        // ขั้นที่ 2: รอหุ่นถึง Home จริงๆ (goal-result: SUCCEEDED)
        console.log('[Shutdown] goHome accepted, waiting for robot to arrive...');
        showCancellable('กำลังเดินทางกลับ Home...');

        const goalResult = await Promise.race([
                new Promise((resolve) => {
                    window.electronAPI.onGoalResultOnce((data) => resolve(data)); // ← once
                }),
                delay(HOME_TIMEOUT_MS).then(() => ({ status: 'TIMEOUT' }))
            ]);

        if (_cancelRequested) return;

        if (goalResult.status === 'SUCCEEDED') {
            console.log('[Shutdown] Robot arrived at Home.');
            showCancellable('ถึง Home แล้ว กำลัง Shutdown...');
            await delay(1500);
        } else {
            console.warn(`[Shutdown] Goal ended with: ${goalResult.status}`);
            showCancellable(`⚠️ หุ่นไม่ถึง Home (${goalResult.status})\nกำลัง Shutdown ต่อ...`);
            await delay(2500);
        }
    }
} catch (err) {
    if (_cancelRequested) return;
    console.error('[Shutdown] goHome error:', err);
    showCancellable('⚠️ เกิดข้อผิดพลาด\nกำลัง Shutdown ต่อ...');
    await delay(2500);
}

    if (_cancelRequested) return;

    startCountdown(doShutdown);
}

// ---- Shutdown ----
async function doShutdown() {
    console.log(`[Shutdown] ${new Date().toISOString()} - Executing shutdown.`);

    showStatus('กำลังบันทึกตำแหน่ง...');
    try {
        await window.electronAPI.stopNavigation(true); // true = save pose
        await delay(1000);
    } catch (err) {
        console.warn('[Shutdown] stopNavigation error (ignored):', err);
    }

    showStatus('กำลัง Power Off Raspberry Pi...');
    console.log(`[Shutdown] ${new Date().toISOString()} - Sending shutdown_raspi command.`);
    window.electronAPI.sendCommand('shutdown_raspi');
    // รอสักครู่แล้วปิด overlay — Pi จะ disconnect เอง
    await delay(2000);
    hideOverlay();
    _isShuttingDown = false;

}

// ---- Countdown ----
function startCountdown(onDone, seconds = 5) {
    let remaining = seconds;

        function tick() {
        if (remaining <= 0) {                     // check ก่อน
            clearInterval(_countdownTimer);
            _countdownTimer = null;
            hideOverlay();
            onDone();
            return;
        }
        showStatus(`⚠️ Power Off Raspberry Pi ภายใน ${remaining} วินาที...`, {
            showCancel: true,
            onCancel: () => { _resetState(); },
        });
        remaining--;
    }

    tick();
    _countdownTimer = setInterval(tick, 1000);
}

// ---- UI Helpers ----
function getOrCreateOverlay() {
    let overlay = document.getElementById('shutdown-overlay');
    if (!overlay) {
        overlay = document.createElement('div');
        overlay.id = 'shutdown-overlay';
        overlay.style.cssText = `
            position: fixed; inset: 0; z-index: 9999;
            background: rgba(0,0,0,0.80);
            display: flex; align-items: center; justify-content: center;
        `;
        document.body.appendChild(overlay);
    }
    return overlay;
}

function hideOverlay() {
    document.getElementById('shutdown-overlay')?.remove();
}

// showDialog รองรับปุ่มที่ 3 (dismissText/onDismiss) สำหรับ "ยกเลิก"
function showDialog({ icon = '⚠️', title, message, confirmText, cancelText, dismissText, onConfirm, onCancel, onDismiss }) {
    const overlay = getOrCreateOverlay();
    overlay.innerHTML = `
        <div style="
            background: #1e1e2e; border-radius: 12px; padding: 32px 40px;
            color: #fff; text-align: center; max-width: 400px; width: 90%;
            box-shadow: 0 8px 32px rgba(0,0,0,0.6);
        ">
            <div style="font-size:32px; margin-bottom:10px;">${icon}</div>
            ${title ? `<div style="font-weight:700; font-size:16px; margin-bottom:12px;">${title}</div>` : ''}
            <p style="font-size:15px; margin-bottom:28px; line-height:1.7; white-space:pre-line;">${message}</p>
            <div style="display:flex; flex-direction:column; gap:10px; align-items:center;">
                <div style="display:flex; gap:10px; justify-content:center; width:100%;">
                    <button id="sd-confirm" style="
                        flex:1; padding:10px 16px; border-radius:8px; border:none; cursor:pointer;
                        background:#4CAF50; color:#fff; font-size:14px; font-weight:600;
                    ">${confirmText}</button>
                    <button id="sd-cancel" style="
                        flex:1; padding:10px 16px; border-radius:8px; border:none; cursor:pointer;
                        background:#e53935; color:#fff; font-size:14px; font-weight:600;
                    ">${cancelText}</button>
                </div>
                ${dismissText ? `
                <button id="sd-dismiss" style="
                    padding:8px 32px; border-radius:8px; cursor:pointer;
                    background:transparent; border:1px solid #555;
                    color:#aaa; font-size:13px;
                ">${dismissText}</button>` : ''}
            </div>
        </div>
    `;
    document.getElementById('sd-confirm').onclick = () => { hideOverlay(); onConfirm(); };
    document.getElementById('sd-cancel').onclick  = () => { hideOverlay(); onCancel();  };
    if (dismissText && onDismiss) {
        document.getElementById('sd-dismiss').onclick = () => { hideOverlay(); onDismiss(); };
    }
}

function showStatus(message, { showCancel = false, onCancel } = {}) {
    const overlay = getOrCreateOverlay();
    overlay.innerHTML = `
        <div style="
            background: #1e1e2e; border-radius: 12px; padding: 32px 40px;
            color: #fff; text-align: center; max-width: 380px; width: 90%;
            box-shadow: 0 8px 32px rgba(0,0,0,0.6);
        ">
            <p style="font-size:16px; margin-bottom:${showCancel ? '24px' : '0'}; line-height:1.6;">
                ${message}
            </p>
            ${showCancel ? `
                <button id="sd-abort" style="
                    padding:10px 28px; border-radius:8px; border:none; cursor:pointer;
                    background:#555; color:#fff; font-size:14px;
                ">ยกเลิก</button>
            ` : ''}
        </div>
    `;
    if (showCancel && onCancel) {
        document.getElementById('sd-abort').onclick = onCancel;
    }
}

function delay(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}