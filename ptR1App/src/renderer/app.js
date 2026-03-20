console.log('app.js started');

import { setNavRunning } from './modules/shutdownManager.js';
import { initRelayButtons } from './modules/relayControl.js';
import { CanvasRecorder } from './modules/recorder.js';

import {renderObjects, renderScan, initStaticMap, renderAllLayers, cancelMode} from './modules/mapStatic.js';
import { renderDashboardMap, initHomeMap,startRenderLoop,stopRenderLoop,resetViewV2} from './modules/mapHome.js';

import { setupVideoPlayer } from './modules/videoPlayer.js';

import { updateRobotPose } from './modules/robotState.js';
import { setPlannedPath } from './modules/planState.js';
import * as patrolState from './modules/patrolState.js';

import * as patrol from './modules/patrol.js';

import {resetLiveMapView,processLiveMapData, initLiveMap, drawLiveMap, updateLiveRobotPose } from './modules/mapLive.js';
import * as mapView from './modules/mapView.js'; 
import { updateLaserScan } from './modules/laserScanState.js';

import { initInputControl } from './modules/inputControl.js';
import { initProfileManager } from './modules/profileManager.js';
import { initSlamControl } from './modules/slamControl.js';
import { RobotStatusRenderer, PidTuner,PiSystemRenderer} from './modules/robotStatusView.js';
import { initDetectionSettings } from './modules/detectionSettings.js';
import { initShutdownManager } from './modules/shutdownManager.js';
import { activeMap } from './modules/mapState.js';


let recorder = null;
let isFirstMapReceived = false; // ตัวแปรเพื่อตรวจสอบว่าได้รับข้อมูลแผนที่ครั้งแรกหรือยัง
let lastFrameTime = 0;
const targetFPS = 15; //live map  Frame Rate 
const fpsInterval = 1000 / targetFPS;
let liveMapRenderId = null;
let isHomeMapInitialized = false;
let isTfRenderPending = false;
const rosDependentButtons = [
  'delete-map-btn',
  'sync-maps-btn',
  'save-map-btn',
  'start-slam-btn',
  'set-pose-btn',
  'set-home-btn',
  'go-home-btn',
  'start-nav-btn',
  'stop-nav-btn'
  ];

document.addEventListener('DOMContentLoaded', async() => {
  console.log("app: DOMContentLoaded fired!");
  switchView('home')
  initInputControl();       // จัดการ Keyboard/Servo/Speed
  await initProfileManager(); // จัดการ Profile/Connect
  initSlamControl();        // จัดการ SLAM ปุ่มต่างๆ
  patrol.initPatrolManager();
  requestAnimationFrame(renderLoop);
  setupPatrolEvents();
  initRelayButtons();
  setupRecorder();
  setupGlobalCallbacks();
  setupVideoPlayer();
  initDetectionSettings();
  initShutdownManager(() => activeMap);
  //ตั้งค่าการสลับ View ผ่าน Sidebar
  document.querySelectorAll('.sidebar-item').forEach(item => {
    item.addEventListener('click', () => switchView(item.dataset.view));
  });
  setupMapToggles();
  const mapWrapper = document.querySelector('.map-wrapper');
    if (mapWrapper) {
      //ส่ง renderAllLayers เป็น callback สำหรับการ Pan/Zoom
      mapView.initMapViewController(
        mapWrapper,
        renderAllLayers, // Callback สำหรับ Static Map
        drawLiveMap      // Callback สำหรับ Live Map
      );
    }
    resetLiveMapView();
  document.getElementById('reset-live-view-btn').addEventListener('click', () => {
    resetLiveMapView();
  });
  const resetHomeBtn = document.getElementById('reset-home-view-btn');
  if (resetHomeBtn) {
    resetHomeBtn.addEventListener('click', () => {
      resetViewV2();
    });
  }

  const statusRenderer = new RobotStatusRenderer();
  const piSystemRenderer = new PiSystemRenderer();
  const pidTuner = new PidTuner();
  
  if (window.electronAPI && window.electronAPI.onRobotStatus) {
        window.electronAPI.onRobotStatus((dataString) => {
            statusRenderer.update(dataString);
            pidTuner.updateFromStatus(dataString); 
        });
    } else {
        console.warn("window.api.onRobotStatus not found");
    }
  if(window.electronAPI && window.electronAPI.onSystemPi) {
        window.electronAPI.onSystemPi((data) => {
            piSystemRenderer.update(data);
        });
    } else {
        console.warn("window.api.onSystemPi not found");
    }
  });
// ตอน nav start สำเร็จ
const result = await window.electronAPI.startNavigation(true);
if (result.success) {
    setNavRunning(true);
}

// ตอน nav stop
await window.electronAPI.stopNavigation(true);
setNavRunning(false);


function updateRosButtons(isConnected) {
        rosDependentButtons.forEach(id => {
            const btn = document.getElementById(id);
            if (btn) {
                btn.disabled = !isConnected;
                // เพิ่ม/ลดความทึบแสงเพื่อให้รู้ว่ากดไม่ได้ (Optional)
                btn.style.opacity = isConnected ? '1' : '0.5';
                btn.style.cursor = isConnected ? 'pointer' : 'not-allowed';
            }
        });
    }
// --- Helper Functions ---
function setupMapToggles() {
    document.getElementById('btn-static-map').addEventListener('click', () => {
        toggleMapLayers(true);
        stopLiveMapRender();
    });
    document.getElementById('btn-live-map').addEventListener('click', () => {
        toggleMapLayers(false);
        startLiveMapRender();
    });
}
function toggleMapLayers(isStatic) {
    const staticGroup = ['map-background-layer', 'map-objects-layer', 'map-scan-layer'];
    const staticBtn = document.getElementById('btn-static-map');
    const liveBtn = document.getElementById('btn-live-map');
    
    staticGroup.forEach(id => document.getElementById(id).classList.toggle('hidden', !isStatic));
    document.getElementById('liveMapCanvas').classList.toggle('hidden', isStatic);
    
    staticBtn.classList.toggle('active', isStatic);
    liveBtn.classList.toggle('active', !isStatic);
    
    document.getElementById('static-control-box').classList.toggle('hidden', !isStatic);
    document.getElementById('live-control-box').classList.toggle('hidden', isStatic);
    document.getElementById('patrol-status-label').classList.toggle('hidden', !isStatic);
    document.querySelector('.canvas-controls').classList.toggle('hidden', !isStatic);
}
function setupPatrolEvents() {
    document.getElementById('start-patrol-btn').addEventListener('click', patrol.startPatrol);
    document.getElementById('pause-patrol-btn').addEventListener('click', patrol.pausePatrol);
    document.getElementById('resume-patrol-btn').addEventListener('click', patrol.resumePatrol);
    document.getElementById('stop-patrol-btn').addEventListener('click', patrol.stopPatrol);
    document.getElementById('loop-patrol-checkbox').addEventListener('change', (e) => patrolState.setLooping(e.target.checked));
    console.log('Patrol: Event listeners set up.');
}
function setupRecorder() {
    const canvas = document.getElementById('capture-canvas');
    const video = document.getElementById('stream');
    


    if(canvas && video) {
        // 1. เริ่มต้น Recorder
        
        recorder = new CanvasRecorder(canvas, { fps: 12, segmentMs: 10 * 60 * 1000 });
        
        const startBtn = document.getElementById('start-record');
        const stopBtn = document.getElementById('stop-record');
        const splitTimeInput = document.getElementById('record-split-time');

        window.electronAPI.onVideoSaveStatus((result) => {
            const original = stopBtn.textContent;
            if (result.success) {
                stopBtn.textContent = 'Saved!';
                stopBtn.style.background = '#2a7a2a';
            } else {
                stopBtn.textContent = 'Save Failed';
                stopBtn.style.background = '#7a2a2a';
            }
            setTimeout(() => {
                stopBtn.textContent = original;
                stopBtn.style.background = '';
            }, 3000); // คืนค่าเดิมใน 3 วินาที
        });
        
        startBtn.addEventListener('click', () => { 
    if (splitTimeInput) {
        let mins = parseInt(splitTimeInput.value);
        if (isNaN(mins) || mins < 1) mins = 10; 
        splitTimeInput.value = mins;
        recorder.segmentMs = mins * 60 * 1000;
            }

            recorder.start(); 
            startBtn.disabled = true; 
            stopBtn.disabled = false; 
            splitTimeInput.disabled = true;
            startBtn.textContent = '🔴 Recording...'; 
        });

        stopBtn.addEventListener('click', () => { 
          recorder.stop(); 
          startBtn.disabled = false; 
          stopBtn.disabled = true; 
          splitTimeInput.disabled = false;
          startBtn.textContent = 'Start Recording'; 
      });
        const ctx = canvas.getContext('2d');
        const drawLoop = () => {
            if (video.readyState === video.HAVE_ENOUGH_DATA) {
                if (canvas.width !== video.videoWidth || canvas.height !== video.videoHeight) {
                    canvas.width  = video.videoWidth  || 640;
                    canvas.height = video.videoHeight || 480;
                    }
                // วาดวิดีโอเป็นพื้นหลัง
                ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
            }
            requestAnimationFrame(drawLoop);
        };
        drawLoop(); // สั่งรัน Loop ทันที

        console.log('Recorder: Canvas recorder & Draw loop initialized.');
    }
}
function setupGlobalCallbacks() {
    // ROS Connection Status
    const rosStatusEl = document.getElementById('home-ros-status');
    rosStatusEl.textContent = 'Disconnected';          // ← set ก่อนรอ event
    rosStatusEl.className = 'status-disconnected';
    updateRosButtons(false);

    window.electronAPI.onConnectionStatus((status) => {
    rosStatusEl.textContent = status.message;
    rosStatusEl.className = status.connected
        ? 'status-connected'
        : (status.connecting ? 'status-connecting' : 'status-disconnected');
    updateRosButtons(status.connected);
});
    
    
    window.electronAPI.onStreamStatus((res) => console.log("Stream:", res));

    // Manual Mode Toggle
    document.getElementById('keyboard-toggle').addEventListener('change', (e) => {
        document.getElementById('mode-label').textContent = e.target.checked ? 'MANUAL ON' : 'MANUAL OFF';
        window.electronAPI.setManualMode(e.target.checked);
    });

    // SLAM / Map Updates
    window.electronAPI.onLiveMap((mapData) => {
        const liveCanvas = document.getElementById('liveMapCanvas');
        const isLiveMapVisible = liveCanvas && !liveCanvas.classList.contains('hidden');
        // ถ้าหน้า Live Map ไม่ได้แสดงอยู่ ให้ข้ามการประมวลผลไปเลย (Save CPU)
        if (!isLiveMapVisible) return;
        
        processLiveMapData(mapData);
        if (liveCanvas && !liveCanvas.classList.contains('hidden')) {
            if (!isFirstMapReceived) { 
                resetLiveMapView(); 
                isFirstMapReceived = true; 
            }
        }
    });
    window.electronAPI.onTfUpdate((tfData) => {
        // TF ส่งมาเป็น { translation: {x,y,z}, rotation: {x,y,z,w} }
        const position = { 
            x: tfData.translation.x, 
            y: tfData.translation.y 
        };
        const orientation = tfData.rotation;

        // เรียกฟังก์ชันวาดหุ่นยนต์ตัวเดิมของคุณ
        const pose = { position, orientation };
        updateRobotPose(position, orientation);
        updateLiveRobotPose(pose)
        
        if (!isTfRenderPending) {
        isTfRenderPending = true;

        requestAnimationFrame(() => {
            // วาดจริงตรงนี้ (จะทำงานเมื่อหน้าจอพร้อม Refresh)
            renderDashboardMap(); // วาด Mini Map
            renderObjects();      // วาด Main Map (หุ่นยนต์)
            
            // ปลดล็อค เพื่อให้รอบหน้าสั่งวาดใหม่ได้
            isTfRenderPending = false;
            });
         }
    });



    window.electronAPI.onPlannedPath(setPlannedPath);
    window.electronAPI.onLaserScan((scan) => {
        updateLaserScan(scan);
        renderDashboardMap();
        if (!document.getElementById('map-scan-layer').classList.contains('hidden')) renderScan();
    });
    window.electronAPI.onPatrolStatusChange((status) => {
        console.log(`ROS Patrol Status: ${status}`);
        if (status === 'active' || status === 'patrolling') {
            patrolState.setPatrolling(true);
            patrolState.updateStatus("Patrolling"); // อัปเดตข้อความบนจอ
        } else {
            patrolState.setPatrolling(false);
            patrolState.updateStatus("Idle");
        }
    });
    console.log('app: Global callbacks set up.');
}

function renderLoop(currentTime) {
  // Get the canvas element.
  const liveMapCanvas = document.getElementById('liveMapCanvas');

  // SELF-STOPPING GUARD: If the canvas is hidden or doesn't exist,
  // stop the render loop immediately by not requesting the next frame.
  if (!liveMapCanvas || liveMapCanvas.classList.contains('hidden')) {
    liveMapRenderId = null; // Ensure the state reflects that the loop is stopped.
    return;
  }

  // Continue the loop by requesting the next animation frame.
  liveMapRenderId = requestAnimationFrame(renderLoop);
  
  // Throttle the drawing to the specified FPS.
  const elapsed = currentTime - lastFrameTime;
  if (elapsed > fpsInterval) {
    // Adjust lastFrameTime for more accurate throttling.
    lastFrameTime = currentTime - (elapsed % fpsInterval);
    
    // Draw the map. No need to check for visibility again.
    drawLiveMap(); 
  }
}

function stopLiveMapRender() {
  // ถ้ามีการเรนเดอร์อยู่ ให้หยุด
  if (liveMapRenderId) {
    console.log("Stopping Live Map render loop.");
    cancelAnimationFrame(liveMapRenderId);
    liveMapRenderId = null;
  }
}
function startLiveMapRender() {
  // Only start a new loop if one isn't already running.
  if (!liveMapRenderId) {
    console.log("Starting Live Map render loop.");
    // Initialize the timer to start throttling correctly from the first frame.
    lastFrameTime = performance.now();
    // Use requestAnimationFrame to start the loop smoothly.
    liveMapRenderId = requestAnimationFrame(renderLoop);
  }
}

function switchView(viewName) {
  // สั่งรีเซ็ตโหมดวาด/เล็งเป้า ทันทีที่เปลี่ยนหน้า
  cancelMode();

  // ซ่อนทุก View และเอา active ออกจาก sidebar
  document.querySelectorAll('.view').forEach(view => view.classList.add('hidden'));
  document.querySelectorAll('.sidebar-item').forEach(item => item.classList.remove('active'));

  // แสดง View และ Sidebar item ที่ต้องการ
  const activeView = document.getElementById(`view-${viewName}`);
  const activeSidebarItem = document.querySelector(`.sidebar-item[data-view="${viewName}"]`);

  if (activeView) activeView.classList.remove('hidden');
  if (activeSidebarItem) activeSidebarItem.classList.add('active');
  
  // --- Logic การ Init ของแต่ละหน้า ---
  if (viewName === 'home') {
      const homeCanvas = document.getElementById('homeMapCanvas');
      if (homeCanvas) {
          // เช็คว่าเคย Init หรือยัง?
          requestAnimationFrame(() => {
             if (!isHomeMapInitialized) {
                 initHomeMap(homeCanvas);
                 isHomeMapInitialized = true;
             }
             startRenderLoop(); 
          });
      }
  } else {
      //ถ้าไม่ใช่หน้า Home ให้หยุดวาดเพื่อประหยัดเครื่อง
      stopRenderLoop();
  }
  // กรณีหน้า Map
  if (viewName === 'map') {
    initStaticMap();
    
    //ถ้าปุ่ม Live Map Active อยู่ ให้เริ่ม Loop ใหม่
    const liveBtn = document.getElementById('btn-live-map');
    if (liveBtn && liveBtn.classList.contains('active')) {
        // ต้องเอา class hidden ออกก่อนเรียก start ไม่งั้น loop จะ kill ตัวเองอีก
        const liveMapCanvas = document.getElementById('liveMapCanvas');
        if(liveMapCanvas) liveMapCanvas.classList.remove('hidden');
        
        if (typeof initLiveMap === 'function') initLiveMap();
        startLiveMapRender(); // สั่งเริ่ม Loop ใหม่
    }
  }
}



// ============================================================
// Capture Panel
// ============================================================
(function initCapturePanel() {
  const elLabel    = document.getElementById('capture-label');
  const elInterval = document.getElementById('capture-interval');
  const elStatus   = document.getElementById('capture-status');
  const elCount    = document.getElementById('capture-count');
  const elPreview  = document.getElementById('capture-preview');
  const elLastLbl  = document.getElementById('capture-last-label');
  const btnSingle  = document.getElementById('capture-single');
  const btnBurst   = document.getElementById('capture-burst');
  const btnStop    = document.getElementById('capture-stop');
  const btnFolder  = document.getElementById('capture-open-folder');

  // โหลด stats ตอนเปิด
  window.electronAPI.captureGetStats().then(res => {
    if (res.success && res.labels.length > 0) {
      const total = res.labels.reduce((s, l) => s + l.count, 0);
      elCount.textContent = `${total} images`;
    }
  });

  // Single Shot
  btnSingle.addEventListener('click', async () => {
    const label = elLabel.value.trim() || 'object';
    elStatus.textContent = '📷 Capturing...';
    await window.electronAPI.captureSnapshot(label);
  });

  // Start Burst
  btnBurst.addEventListener('click', async () => {
    const label    = elLabel.value.trim() || 'object';
    const interval = parseFloat(elInterval.value) || 2.0;
    btnBurst.disabled = true;
    btnStop.disabled  = false;
    elStatus.textContent = `▶ Bursting every ${interval}s — label: "${label}"`;
    await window.electronAPI.captureStartBurst({ label, interval });
  });

  // Stop Burst
  btnStop.addEventListener('click', async () => {
    btnBurst.disabled = false;
    btnStop.disabled  = true;
    elStatus.textContent = '■ Stopped';
    await window.electronAPI.captureStopBurst();
  });

  // Open Folder
  btnFolder.addEventListener('click', async () => {
    const label = elLabel.value.trim() || '';
    await window.electronAPI.captureOpenFolder(label);
  });

  // รับผลจาก ROS (capture:result)
  window.electronAPI.onCaptureResult((data) => {
    elCount.textContent   = `${data.count} images`;
    elLastLbl.textContent = data.label || '—';
    elStatus.textContent  = `✅ Saved: ${data.filename}`;
    if (data.filepath) {
      elPreview.src = `file://${data.filepath}`;
      elPreview.classList.remove('hidden');
    }
  });
})();