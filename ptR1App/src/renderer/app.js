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
const targetFPS = 1; //live map  Frame Rate 
const fpsInterval = 1000 / targetFPS;
let liveMapRenderId = null;
let isHomeMapInitialized = false;
const rosDependentButtons = [
  'delete-map-btn',
  'sync-maps-btn',
  'save-map-btn',
  'start-slam-btn',
  'set-pose-btn',
  'set-home-btn',
  'go-home-btn',
  'start-nav-btn'
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
  const videoElement = document.getElementById('stream');
  if (videoElement) {
      window.overlay = new OverlayCanvas('yolo-overlay', videoElement);
      console.log("Overlay initialized globally.");
  }

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
  
  //Detection Schedule Elements
  const scheduleToggle = document.getElementById('schedule-toggle');
  const startTimeInput = document.getElementById('alert-start-time');
  const endTimeInput = document.getElementById('alert-end-time');
  const setScheduleBtn = document.getElementById('update-schedule-btn');
  function updateSecuritySchedule() {
    const start = startTimeInput.value;
    const end = endTimeInput.value;
    const isEnabled = scheduleToggle.checked;

    //ดึงค่าจาก Checkbox ทั้งหมดที่มี class 'sec-item-check'
    const checkboxes = document.querySelectorAll('.sec-item-check');
    const selectedItems = [];
    checkboxes.forEach(cb => {
        if (cb.checked) {
            selectedItems.push(cb.value); // เก็บค่า value (เช่น 'person', 'cat')
        }
    });

    // 2. ส่งค่าไปอัปเดต Overlay
    if (window.overlay) {
        window.overlay.setRestrictedTime(start, end, isEnabled);
        window.overlay.setRestrictedItems(selectedItems); // ส่ง Array รายการที่เลือกไป
    }
    
    return selectedItems; // Return ไว้ log เล่นๆ
    }
    if (setScheduleBtn) {
        setScheduleBtn.addEventListener('click', () => {
            const items = updateSecuritySchedule();
            alert(`Settings Updated!\nActive: ${scheduleToggle.checked}\nTime: ${startTimeInput.value} - ${endTimeInput.value}\nAlert Objects: ${items.join(', ')}`);
        });
    }
    // อัปเดตทันทีเมื่อกด Toggle
    if (scheduleToggle) {
        scheduleToggle.addEventListener('change', updateSecuritySchedule);
    }
    // (Optional) อัปเดตทันทีเมื่อติ๊กเลือกของ ก็ทำได้เช่นกัน
    document.querySelectorAll('.sec-item-check').forEach(cb => {
        cb.addEventListener('change', updateSecuritySchedule);
    });


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
    if(canvas) {
        recorder = new CanvasRecorder(canvas, { fps: 30, segmentMs: 10 * 60 * 1000 });
        const startBtn = document.getElementById('start-record');
        const stopBtn = document.getElementById('stop-record');
        
        startBtn.addEventListener('click', () => { recorder.start(); startBtn.disabled = true; stopBtn.disabled = false; });
        stopBtn.addEventListener('click', () => { recorder.stop(); startBtn.disabled = false; stopBtn.disabled = true; });
        console.log('Recorder: Canvas recorder initialized.');
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

    window.electronAPI.onRobotPosSlam((pose) => pose.position && updateLiveRobotPose(pose));
    window.electronAPI.onRobotPosAmcl((pose) => {
        if (pose.position) {
            updateRobotPose(pose.position, pose.orientation);
            renderDashboardMap();
            renderObjects();
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
            setPatrolling(true);
            updateStatus("Patrolling"); // อัปเดตข้อความบนจอ
        } else {
            setPatrolling(false);
            updateStatus("Idle");
        }
    });
    console.log('app: Global callbacks set up.');
}

function renderLoop(currentTime) {
  // Get the canvas element.
  const liveMapCanvas = document.getElementById('liveMapCanvas');

  // 🛑 SELF-STOPPING GUARD: If the canvas is hidden or doesn't exist,
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
    if (typeof initLiveMap === 'function') initLiveMap();
  }
}
