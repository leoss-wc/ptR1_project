// modules/shutdownManager.js

// ---- Config ----
const HOME_TIMEOUT_MS     = 600000; // 600s timeout สำหรับ goHome
const LOW_BAT_WARN_PCT    = 20;    // แสดง suggestion ครั้งแรกที่ 20%
const LOW_BAT_URGENT_PCT  = 8;    // บังคับกลับ home อัตโนมัติที่ 8%
const BAT_NOTIFY_COOLDOWN = 5 * 60 * 1000; // แสดง suggestion ซ้ำได้ทุก 5 นาที

// ---- State ----
let _isShuttingDown    = false;
let _countdownTimer    = null;
let _rosConnected      = false;
let _lastBatNotifyTime = 0;      // ป้องกัน notify ถี่เกินไป
let _urgentTriggered   = false;  // บังคับกลับ home ครั้งเดียวเท่านั้น

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

// ---- Battery Monitor ----
function _handleBatteryStatus(str, getActiveMap) {
    if (!str || !_rosConnected) return;

    const batMatch = str.match(/Bat:([\d\.]+)V/);
    if (!batMatch) return;

    const voltage = parseFloat(batMatch[1]);
    const percent = getBatteryPercent(voltage);

    // --- Urgent: บังคับกลับ home อัตโนมัติ ---
    if (percent <= LOW_BAT_URGENT_PCT && !_urgentTriggered && !_isShuttingDown) {
        _urgentTriggered = true;
        console.warn(`[Shutdown] Battery CRITICAL: ${percent}% (${voltage}V). Auto go home!`);
        showUrgentBatteryAlert(percent, voltage, getActiveMap);
        return;
    }

    // --- Warning: แสดง suggestion ---
    if (percent <= LOW_BAT_WARN_PCT && percent > LOW_BAT_URGENT_PCT && !_isShuttingDown) {
        const now = Date.now();
        if (now - _lastBatNotifyTime < BAT_NOTIFY_COOLDOWN) return; // cooldown
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
            <span style="font-size:22px;">🔋</span>
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
        goHomeAndShutdown(getActiveMap);
    };

    // auto-dismiss หลัง 30 วินาที
    setTimeout(() => el?.remove(), 30000);
}

// --- Urgent: แสดง dialog บังคับ (ไม่มีปุ่ม dismiss) ---
function showUrgentBatteryAlert(percent, voltage, getActiveMap) {
    if (document.getElementById('bat-suggestion')) {
        document.getElementById('bat-suggestion').remove();
    }

    showDialog({
        icon: '🔴',
        title: 'Low Battery - Returning Home',
        message: `Battery at ${percent}% (${voltage.toFixed(2)}V)\nThe robot will attempt to return Home before powering off.`,
        confirmText: 'Go Home Immediately',
        cancelText: 'Cancel (Not Recommended)',
        onConfirm: () => goHomeAndShutdown(getActiveMap),
        onCancel:  () => {
            _urgentTriggered = false; // อนุญาตให้ trigger ซ้ำได้ถ้า user ยกเลิก
            console.warn('[Shutdown] User cancelled urgent battery go-home.');
        },
    });
}

// ---- Manual Shutdown Flow ----
function startShutdownFlow(getActiveMap) {
    if (_isShuttingDown) {
        console.warn('[Shutdown] Already in shutdown flow.');
        return;
    }
    _isShuttingDown = true;

    // ปิด suggestion ถ้าแสดงอยู่
    document.getElementById('bat-suggestion')?.remove();

    console.log(`[Shutdown] ${new Date().toISOString()} - User initiated shutdown.`);

    if (!_rosConnected) {
        console.warn('[Shutdown] ROS not connected.');
        return;
    }

    showDialog({
        message: 'Are you sure you want to shutdown the robot?\n\nThe robot will attempt to return Home before powering off.',
        confirmText: 'Yes, Go Home',
        cancelText: 'Power Off Now',
        onConfirm: () => goHomeAndShutdown(getActiveMap),
        onCancel:  () => startCountdown(doShutdown),
    });
}

// ---- goHome Logic ----
async function goHomeAndShutdown(getActiveMap) {
    _isShuttingDown = true;
    const activeMap = getActiveMap();

    console.log(`[Shutdown] ${new Date().toISOString()} - Going home. Map: ${activeMap?.name ?? 'unknown'}`);

    if (!activeMap?.name) {
        console.warn('[Shutdown] No active map. Skipping goHome.');
        startCountdown(doShutdown);
        return;
    }

    // 1. หยุด Patrol ก่อน
    try {
        showStatus('Stopping Patrol...');
        await window.electronAPI.stopPatrol();
        await delay(500);
    } catch (err) {
        console.warn('[Shutdown] stopPatrol error (ignored):', err);
    }

    // 2. goHome พร้อม Timeout
    showStatus('Going Home...');
    try {
        const result = await Promise.race([
            window.electronAPI.goHome(activeMap.name),
            delay(HOME_TIMEOUT_MS).then(() => ({ success: false, message: 'Timeout (60s)' }))
        ]);

        if (!result?.success) {
            console.warn(`[Shutdown] goHome failed: ${result?.message}`);
            showStatus(`Cannot go home: ${result?.message ?? 'unknown'}`);
            await delay(2000);
        } else {
            console.log('[Shutdown] goHome succeeded.');
        }
    } catch (err) {
        console.error('[Shutdown] goHome error:', err);
        showStatus('Something went wrong while going home.');
        await delay(2000);
    }

    startCountdown(doShutdown);
}

// ---- Shutdown ----
async function doShutdown() {
    console.log(`[Shutdown] ${new Date().toISOString()} - Executing shutdown.`);

    showStatus('Attempting position save  ...');
    try {
        await window.electronAPI.stopNavigation(true); // true = save pose
        await delay(1000);
    } catch (err) {
        console.warn('[Shutdown] stopNavigation error (ignored):', err);
    }

    showStatus('⏻ กำลัง Power Off Raspberry Pi...');
    console.log(`[Shutdown] ${new Date().toISOString()} - Sending shutdown_raspi command.`);
    window.electronAPI.sendCommand('shutdown_raspi');
}

// ---- Countdown ----
function startCountdown(onDone, seconds = 5) {
    let remaining = seconds;

    function tick() {
        showStatus(`⚠️ Power Off Raspberry Pi ภายใน ${remaining} วินาที...`, {
            showCancel: true,
            onCancel: cancelCountdown,
        });
        if (remaining <= 0) {
            clearInterval(_countdownTimer);
            _countdownTimer = null;
            hideOverlay();
            onDone();
            return;
        }
        remaining--;
    }

    tick();
    _countdownTimer = setInterval(tick, 1000);
}

function cancelCountdown() {
    if (_countdownTimer) {
        clearInterval(_countdownTimer);
        _countdownTimer = null;
    }
    _isShuttingDown  = false;
    _urgentTriggered = false;
    hideOverlay();
    console.log(`[Shutdown] ${new Date().toISOString()} - Cancelled by user.`);
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

function showDialog({ icon = '⚠️', title, message, confirmText, cancelText, onConfirm, onCancel }) {
    const overlay = getOrCreateOverlay();
    overlay.innerHTML = `
        <div style="
            background: #1e1e2e; border-radius: 12px; padding: 32px 40px;
            color: #fff; text-align: center; max-width: 380px; width: 90%;
            box-shadow: 0 8px 32px rgba(0,0,0,0.6);
        ">
            ${title ? `<div style="font-size:28px; margin-bottom:8px;">${icon}</div>
                       <div style="font-weight:700; font-size:16px; margin-bottom:12px;">${title}</div>` : ''}
            <p style="font-size:15px; margin-bottom:28px; line-height:1.7; white-space:pre-line;">${message}</p>
            <div style="display:flex; gap:12px; justify-content:center;">
                <button id="sd-confirm" style="
                    padding:10px 22px; border-radius:8px; border:none; cursor:pointer;
                    background:#4CAF50; color:#fff; font-size:14px; font-weight:600;
                ">${confirmText}</button>
                <button id="sd-cancel" style="
                    padding:10px 22px; border-radius:8px; border:none; cursor:pointer;
                    background:#e53935; color:#fff; font-size:14px; font-weight:600;
                ">${cancelText}</button>
            </div>
        </div>
    `;
    document.getElementById('sd-confirm').onclick = () => { hideOverlay(); onConfirm(); };
    document.getElementById('sd-cancel').onclick  = () => { hideOverlay(); onCancel();  };
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
                    padding:10px 24px; border-radius:8px; border:none; cursor:pointer;
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