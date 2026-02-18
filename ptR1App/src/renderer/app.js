console.log('app.js started');

import { initRelayButtons } from './modules/relayControl.js';
import { CanvasRecorder } from './modules/recorder.js';

import {renderObjects, renderScan, initStaticMap, renderAllLayers, cancelMode} from './modules/mapStatic.js';
import { renderDashboardMap, initHomeMap,startRenderLoop,stopRenderLoop} from './modules/mapHome.js';

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
import { RobotStatusRenderer, PidTuner} from './modules/robotStatusView.js';
import { OverlayCanvas } from './modules/OverlayCanvas.js';


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

  const statusRenderer = new RobotStatusRenderer();
  const pidTuner = new PidTuner();
  
  if (window.electronAPI && window.electronAPI.onRobotStatus) {
        window.electronAPI.onRobotStatus((dataString) => {
            statusRenderer.update(dataString);
            pidTuner.updateFromStatus(dataString); 
        });
    } else {
        console.warn("window.api.onRobotStatus not found");
    }
  
const videoElement = document.getElementById('stream');
    if (videoElement) {
        window.overlay = new OverlayCanvas('yolo-overlay', videoElement);
        console.log("Overlay initialized globally.");
    }

    // --- Security Schedule Elements (New UI) ---
    const modeRadios = document.querySelectorAll('input[name="sec-mode"]');
    const timeBox = document.getElementById('time-settings-box');
    const timeInputs = ['alert-start-time', 'alert-end-time'];
    const updateBtn = document.getElementById('update-security-btn');

    // ฟังก์ชันหลักสำหรับอ่านค่าและส่งไป Overlay
    function applySecuritySettings() {
        if (!window.overlay) return;

        // 1. หาโหมดที่เลือก (Radio)
        // ต้องเช็คก่อนว่ามี element ไหม เพื่อป้องกัน error ถ้า HTML โหลดไม่ทัน
        const radioChecked = document.querySelector('input[name="sec-mode"]:checked');
        if (!radioChecked) return; 
        
        const selectedMode = radioChecked.value;
        const startTime = document.getElementById('alert-start-time').value;
        const endTime = document.getElementById('alert-end-time').value;

        // 2. ดึงรายการ Restricted จาก Checkbox
        const checkboxes = document.querySelectorAll('.sec-item-check');
        const selectedRestricted = [];
        checkboxes.forEach(cb => {
            if (cb.checked) selectedRestricted.push(cb.value);
        });

        // 3. ส่งค่าไปที่ OverlayCanvas ตามโหมด
        switch (selectedMode) {
            case 'disable':
                // ปิดระบบความปลอดภัยทั้งหมด
                window.overlay.setSecurityMode(false);
                break;

            case 'always':
                // เปิดระบบ + ปิดการเช็คเวลา (ให้ทำงานตลอดเวลา)
                window.overlay.setSecurityMode(true);
                window.overlay.setRestrictedTime(startTime, endTime, false); 
                break;

            case 'schedule':
                // เปิดระบบ + เปิดการเช็คเวลา
                window.overlay.setSecurityMode(true);
                window.overlay.setRestrictedTime(startTime, endTime, true); 
                break;
        }

        // อัปเดตรายการของ
        window.overlay.setRestrictedItems(selectedRestricted);
        
        // อัปเดต UI สรุปผล
        updateSummaryDisplay(selectedMode, selectedRestricted);
    }

    function updateSummaryDisplay(mode, restrictedItems) {
        const dangerEl = document.getElementById('summary-danger');
        const restrictedEl = document.getElementById('summary-restricted');
        const timeBox = document.getElementById('time-settings-box');

        // จัดการการแสดงผลช่องเวลา
        if (mode === 'schedule') {
            timeBox.style.opacity = '1';
            timeBox.style.pointerEvents = 'auto';
        } else {
            timeBox.style.opacity = '0.3';
            timeBox.style.pointerEvents = 'none';
        }

        // แสดงรายการ Danger
        if(dangerEl) dangerEl.textContent = "Fire, Knife, Weapon"; 

        // แสดงรายการ Restricted
        if (restrictedEl) {
            if (mode === 'disable') {
                restrictedEl.textContent = "System Disabled";
                restrictedEl.style.color = "#aaa";
            } else {
                restrictedEl.textContent = restrictedItems.length > 0 ? restrictedItems.join(', ') : "None";
                restrictedEl.style.color = "#ccc";
            }
        }
    }

    // --- Event Listeners ---

    // 1. ปุ่ม Update (New UI)
    if (updateBtn) {
        updateBtn.addEventListener('click', () => {
            applySecuritySettings();
            
            // Effect ปุ่มกดแล้วเปลี่ยนสี
            const originalText = updateBtn.innerHTML;
            updateBtn.innerHTML = "Saved!";
            updateBtn.classList.replace('btn-primary', 'btn-success');
            setTimeout(() => {
                updateBtn.innerHTML = originalText;
                updateBtn.classList.replace('btn-success', 'btn-primary');
            }, 1000);
        });
    }

    // 2. เมื่อเปลี่ยนโหมด Radio ให้ Enable/Disable ช่องเวลาทันที (UX)
    modeRadios.forEach(radio => {
        radio.addEventListener('change', () => {
            const isSchedule = (radio.value === 'schedule');
            const timeBox = document.getElementById('time-settings-box');
            if(timeBox) {
                if (isSchedule) {
                    timeBox.style.opacity = '1';
                    timeBox.style.pointerEvents = 'auto';
                } else {
                    timeBox.style.opacity = '0.3';
                    timeBox.style.pointerEvents = 'none';
                }
            }
        });
    });

    // 3. ป้องกันพิมพ์เวลาแล้ว Key ลั่นไปโดนหุ่นยนต์
    timeInputs.forEach(id => {
        const el = document.getElementById(id);
        if (el) {
            el.addEventListener('keydown', (e) => e.stopPropagation());
            el.addEventListener('keyup', (e) => e.stopPropagation());
        }
    });

    // เรียกครั้งแรกเพื่อ Init UI ให้ตรงกับ Default Value
    setTimeout(applySecuritySettings, 500);
  });


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
    document.getElementById('save-path-btn').addEventListener('click', patrol.saveDrawnPath);
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
    const overlayCanvas = document.getElementById('yolo-overlay');

    if(canvas && video && overlayCanvas) {
        // 1. เริ่มต้น Recorder
        recorder = new CanvasRecorder(canvas, { fps: 30, segmentMs: 10 * 60 * 1000 });
        
        const startBtn = document.getElementById('start-record');
        const stopBtn = document.getElementById('stop-record');
        
        startBtn.addEventListener('click', () => { 
            recorder.start(); 
            startBtn.disabled = true; 
            stopBtn.disabled = false; 
        });
        stopBtn.addEventListener('click', () => { 
            recorder.stop(); 
            startBtn.disabled = false; 
            stopBtn.disabled = true; 
        });
        const ctx = canvas.getContext('2d');
        const drawLoop = () => {
            if (video.readyState === video.HAVE_ENOUGH_DATA) {
                // วาดวิดีโอเป็นพื้นหลัง
                ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
                // วาด Overlay ทับ (แม้ overlay จะ display:none ก็วาดติด)
                ctx.drawImage(overlayCanvas, 0, 0, canvas.width, canvas.height);
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
    updateRosButtons(false);
    window.electronAPI.onConnectionStatus((status) => {
        rosStatusEl.textContent = status.message;
        rosStatusEl.className = status.connected ? 'status-connected' : (status.connecting ? 'status-connecting' : 'status-disconnected');
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
        processLiveMapData(mapData);
        if (!isFirstMapReceived) { 
          resetLiveMapView(); isFirstMapReceived = true; 
        }
    });

    //window.electronAPI.onRobotPosSlam((pose) => pose.position && updateLiveRobotPose(pose));
    //window.electronAPI.onRobotPosAmcl((pose) => {});
    window.electronAPI.onTfUpdate((tfData) => {
        // TF ส่งมาเป็น { translation: {x,y,z}, rotation: {x,y,z,w} }
        // เราต้องแปลงให้เข้ากับฟอร์มที่ updateRobotPose ต้องการ
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
