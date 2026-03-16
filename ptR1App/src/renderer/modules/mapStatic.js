// modules/mapStatic.js
import { patrolPath , setGoalPoint, goalPoint} from './patrolState.js';
import { activeMap } from './mapState.js';
import * as mapView from './mapView.js'; 
import { robotPose } from './robotState.js';
import { latestScan } from './laserScanState.js';
import { stopPatrol } from './patrol.js';
import { yawToQuaternion, getYawFromQuaternion } from './utils.js';
import * as patrolState from './patrolState.js';

let backgroundCanvas, backgroundCtx;
let objectsCanvas, objectsCtx;
let scanCanvas, scanCtx;
let interactionCanvas;

let mapImage;
let isDrawing = false;
let isHoveringFirstPoint = false;
let current_map_select = { name: null, base64: null ,meta:null};
let isSettingGoal = false;
let mode = 'none';

let isSettingPose = false; 
let poseStartPosition = null;
let currentMousePos = { x: 0, y: 0 }; //เก็บตำแหน่งเมาส์ล่าสุดบน Canvas

let mapHitCanvas, mapHitCtx; //ตัวแปรสำหรับ Canvas ที่ใช้ตรวจสอบการคลิกแบบ Pixel-perfect

let dimmerMaskImage = null;//ตัวแปรสำหรับเก็บภาพมาสก์ Dimmer ที่สร้างขึ้น

let collisionMapData = null; // เก็บข้อมูล Pixel แผนที่เพื่อเช็คชน
let collisionWidth = 0;
let collisionHeight = 0;
let homePose = null;

let mapEdits = [];

let isStaticMapInitialized = false;

const mapDependentButtons = [
    'toggle-draw-mode',
    'toggle-wall-btn',
    'toggle-eraser-btn',
    'save-edit-btn'
];

export { renderObjects, renderScan };

function updateMapToolsState(hasMap) {
    mapDependentButtons.forEach(id => {
        const btn = document.getElementById(id);
        if (btn) {
            btn.disabled = !hasMap;
            btn.style.opacity = hasMap ? '1' : '0.5';
            btn.style.cursor = hasMap ? 'pointer' : 'not-allowed';
        }
    });
}

export function initStaticMap() {
  // 2. Get element และ context ของทุก Layer
  backgroundCanvas = document.getElementById('map-background-layer');
  objectsCanvas = document.getElementById('map-objects-layer');
  scanCanvas = document.getElementById('map-scan-layer');
  
  // ใช้ Canvas บนสุดเป็นตัวรับ Events ทั้งหมด
  interactionCanvas = scanCanvas; 

  if (!backgroundCanvas || !objectsCanvas || !scanCanvas) {
    console.error("Static map layers not found!");
    return;
  }
  
  backgroundCtx = backgroundCanvas.getContext('2d');
  objectsCtx = objectsCanvas.getContext('2d');
  scanCtx = scanCanvas.getContext('2d');

  if (!isStaticMapInitialized) {
      bindUI();
      setupCanvasEvents();
      loadLocalMapsToGallery();
      loadLastActiveMap();
      updateMapToolsState(false);
      
      isStaticMapInitialized = true;
      console.log("Static Map Initialized (Once)");
  }
  requestAnimationFrame(() => {
      // บังคับให้ระบบคำนวณขนาด (Resize) ใหม่ให้พอดีกับหน้าต่าง
      window.dispatchEvent(new Event('resize'));
  });
}

// ฟังก์ชันสำหรับปรับขนาด Canvas ทั้งหมดให้ตรงกับขนาดของ Container
function renderBackground() {
  if (!mapImage || !backgroundCanvas) return;
  resizeAllCanvases();
  const ctx = backgroundCtx;

  ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
  ctx.imageSmoothingEnabled = false;
  ctx.save();
  ctx.translate(mapView.viewState.offsetX, mapView.viewState.offsetY);
  ctx.scale(mapView.viewState.scale, mapView.viewState.scale);
  ctx.drawImage(mapImage, 0, 0, mapImage.width, mapImage.height);
  ctx.restore();
}
// ฟังก์ชันสำหรับวาด Layer ที่มีวัตถุทั้งหมด (Robot, Path, Goal)
function renderObjects() {
  if (!activeMap.meta || !objectsCanvas) return;
  const ctx = objectsCtx;

  ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
  ctx.imageSmoothingEnabled = false;
  ctx.save();
  ctx.translate(mapView.viewState.offsetX, mapView.viewState.offsetY);
  ctx.scale(mapView.viewState.scale, mapView.viewState.scale);
  //วาด Dimmer Mask ก่อน เพื่อให้พื้นหลังมืดลง
  if (dimmerMaskImage && (mode === 'draw' || mode === 'goal' || mode === 'pose')) {
        ctx.drawImage(dimmerMaskImage, 0, 0, mapImage.width, mapImage.height);
    }

  // วาดทุกอย่างที่ไม่ใช่ Background และ Scan
  drawHome(ctx);
  drawPatrolPath(ctx);
  drawRobot(ctx);
  drawGoal(ctx);
  drawUserWalls(ctx)
  // การวาด Goal จะถูกจัดการผ่าน patrolState และ renderDashboardMap
  
  ctx.restore();

  // วาด UI ที่เป็น Screen-space
  drawInteractionUI(ctx);
}
// ฟังก์ชันสำหรับวาด Layer ที่มี Laser Scan
function renderScan() {
    if (!latestScan || !scanCanvas) return;
    const ctx = scanCtx;

    ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
    ctx.imageSmoothingEnabled = false;
    ctx.save();
    ctx.translate(mapView.viewState.offsetX, mapView.viewState.offsetY);
    ctx.scale(mapView.viewState.scale, mapView.viewState.scale);

    drawLaserScan(ctx);

    ctx.restore();
}
export function renderAllLayers() {
    requestAnimationFrame(() => {
        renderBackground();
        renderObjects();
        renderScan();
    });
}

function resizeAllCanvases() {
    const canvases = [backgroundCanvas, objectsCanvas, scanCanvas];
    canvases.forEach(canvas => {
        if (canvas) {
            canvas.width = canvas.clientWidth;
            canvas.height = canvas.clientHeight;
        }
    });
}

//ฟังก์ชันสำหรับ Reset View โดยใช้หลัก "Fit and Center"
function resetStaticMapView() {
  if (!interactionCanvas || !mapImage) return;

  const canvas = interactionCanvas; // ใช้ canvas บนสุดเป็นตัวอ้างอิงขนาด
  const zoomX = canvas.width / mapImage.width;
  const zoomY = canvas.height / mapImage.height;

  mapView.viewState.scale = Math.min(zoomX, zoomY) * 0.95;
  mapView.viewState.offsetX = (canvas.width - mapImage.width * mapView.viewState.scale) / 2;
  mapView.viewState.offsetY = (canvas.height - mapImage.height * mapView.viewState.scale) / 2;
  
  renderAllLayers();
}

document.getElementById('start-nav-btn').addEventListener('click', async () => {
    if (!activeMap.name || !mapImage) {
        alert("❗ Please select a map from the gallery first.");
        return;
    }
    const btn = document.getElementById('start-nav-btn');
    const originalText = btn.innerHTML;
    
    //ปรับ UI ให้รู้ว่ากำลังทำงาน
    btn.disabled = true;
    btn.innerHTML = `<i class="fas fa-spinner fa-spin"></i> Loading Map...`;

    try {
        console.log(`Selecting Map '${activeMap.name}'...`);
        await window.electronAPI.selectMap(activeMap.name);
        console.log("Starting Navigation...");
        const navRes = await window.electronAPI.startNavigation(true); 
        
        if (!navRes.success) throw new Error(navRes.message);

        //รอ AMCL ตื่น
        btn.innerHTML = `<i class="fas fa-spinner fa-spin"></i> Initializing...`;
        await new Promise(resolve => setTimeout(resolve, 2000));

        console.log(`Attempting Restore for ${activeMap.name}...`);
        /*
        const homeRes = await window.electronAPI.initHome(activeMap.name);

        if (homeRes.success) {
            console.log("Robot initialized at HOME position.");
            //alert(`System Started! Robot is at Home.`);
        } else {
            // กรณีล้มเหลว (เช่น ไม่เคยเซ็ต Home ไว้): ไม่เป็นไร แค่แจ้งเตือน
            console.warn(" Could not init home (Home not set?). User must set pose manually.");
            alert(`System Started. \n Warning: Home location not found.\nPlease set '2D Pose Estimate' manually.`);
        }
            */
    } catch (error) {
        console.error("❌ Error sequence:", error);
        alert(`Error: ${error.message}`);
    } finally {
        // คืนค่าปุ่ม
        btn.disabled = false;
        btn.innerHTML = originalText;
    }
});
const stopNavBtn = document.getElementById('stop-nav-btn');
if (stopNavBtn) {
    stopNavBtn.addEventListener('click', async () => {
        console.log("Stop Nav Button Clicked");
        
        // ใส่ Effect ให้ปุ่มดูเหมือนกำลังทำงาน (Optional)
        stopNavBtn.disabled = true;
        stopNavBtn.innerText = "Stopping...";

        try {
            // เรียก API ผ่าน Bridge
            const result = await window.electronAPI.stopNavigation(true); // true = save pose
            
            if (result.success) {
                console.log("Navigation Stopped Successfully:", result.message);
                // แจ้งเตือนผู้ใช้ (ถ้ามีระบบ notify)
            } else {
                console.error("Failed to stop navigation:", result.message);
            }
        } catch (error) {
            console.error("Error stopping navigation:", error);
        } finally {
            // คืนค่าปุ่มกลับสู่ปกติ
            stopNavBtn.disabled = false;
            stopNavBtn.innerHTML = "Stop nav";
        }
    });
}

function bindUI() {
  // Zoom Controls
  document.getElementById('zoom-in').addEventListener('click', () => {
    if (mapImage) { mapView.viewState.scale *= 1.2; renderAllLayers(); }
  });
  document.getElementById('zoom-out').addEventListener('click', () => {
    if (mapImage) { mapView.viewState.scale /= 1.2; renderAllLayers(); }
  });
  document.getElementById('reset-static-view-btn').addEventListener('click', resetStaticMapView);

  // Map Controls
  document.getElementById('clear-path-btn').addEventListener('click', () => {
    patrolState.patrolPath.length = 0;
    cancelMode();
    renderObjects();
  });
  
  document.getElementById('sync-maps-btn').addEventListener('click', () => {
    window.electronAPI.syncMaps();
  });

  //ปุ่ม Delete Map 
  document.getElementById('delete-map-btn').addEventListener('click', () => {
    if (!current_map_select || !current_map_select.name) {
      alert("Please select a map from the gallery first.");
      return;
    }
    const mapName = current_map_select.name;
    if (confirm(`Are you sure you want to PERMANENTLY delete map "${mapName}"?`)) {
      window.electronAPI.deleteMap(mapName);
      window.electronAPI.deleteMapCache(mapName); // ลบ Cache ที่เกี่ยวข้องด้วย (ถ้ามี)
      setTimeout(() => {
        window.electronAPI.syncMaps(); // รีเฟรช Gallery หลังลบ
        current_map_select = { name: null, base64: null, meta: null };
        renderAllLayers(); // เคลียร์แผนที่ออกจากหน้าจอ
        }, 1000);
    }
  });
  document.getElementById('set-home-btn').addEventListener('click', () => {
    if (!activeMap.name) return
    if (confirm(`Set CURRENT robot position as HOME for map "${activeMap.name}"?`)) {
      window.electronAPI.setHome(activeMap.name);
    }

  });
  document.getElementById('go-home-btn').addEventListener('click', () => {
    if (!activeMap.name) return alert("Please load a map first.");
    window.electronAPI.goHome(activeMap.name);
  });
  document.getElementById('init-home-btn').addEventListener('click', () => {
    if (!activeMap.name) return alert("Please load a map first.");
    window.electronAPI.initHome(activeMap.name);
  });

  window.electronAPI.onHomeResult((res) => {
    if (res.success) {
      if (res.action !== 'Go Home') console.log(`${res.action}: Success`);
      if (res.action === 'Set Home') {
          updateHomePose(); 
      }
      console.log(`[Home] ${res.action}: ${res.message}`);
    } else {
      console.log(`❌ ${res.action} Failed: ${res.message}`);
    }
  });
  // Listener รับผลการลบแผนที่
  window.electronAPI.onMapDeleteResult((result) => {
    if (result.success) {
      alert(`${result.message}`);
      // เคลียร์ Selection
      current_map_select = { name: null, base64: null, meta: null };
      mapImage = null; // เคลียร์รูปภาพ
      // เคลียร์หน้าจอ
      const ctx = backgroundCtx;
      ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
      renderObjects();
      // โหลด Gallery ใหม่
      updateMapToolsState(false);
      loadLocalMapsToGallery();
    } else {
      alert(`Error: ${result.message}`);
    }
  });

  // Mode Toggles
  document.getElementById('set-goal-btn').addEventListener('click', () => {
    toggleMode('goal');
  });
  document.getElementById('set-pose-btn').addEventListener('click', () => {
    toggleMode('pose');
  });
  document.getElementById('toggle-draw-mode').addEventListener('click', () => {
    toggleMode('draw');
  });

  // Map Selection Logic
  document.getElementById('select-map-btn').addEventListener('click', async () => {
    if (!current_map_select.name || !mapImage) {
      alert("Please select a map from the gallery first.");
      return;
    }
    console.log(`Activating map: ${current_map_select.name}`);
    await activateMap(current_map_select.name, current_map_select.meta);
  });
  document.getElementById('toggle-eraser-btn').addEventListener('click', () => toggleMode('eraser'));
  document.getElementById('undo-edit-btn').addEventListener('click', () => undoLastEdit());
  document.getElementById('toggle-wall-btn').addEventListener('click', () => {
    toggleMode('wall');
  });

  document.getElementById('save-edit-btn').addEventListener('click', async () => {
    const defaultName = activeMap.name + "_v2";
    let newName = await showPrompt("Enter name for the new map:", defaultName);

    if (!newName) return;
    newName = newName.trim();

    // เช็คชื่อซ้ำ
    const existingNames = await getExistingMapNames();
    if (existingNames.includes(newName)) {
        const confirmOverwrite = confirm(`⚠️ Map named "${newName}" already exists!\nDo you want to OVERWRITE it?`);
        if (!confirmOverwrite) return;
        
        //ถ้าตกลงเขียนทับ -> ต้องลบ Cache เก่าทิ้งก่อน
        console.log(`Clearing cache for overwritten map: ${newName}`);
        await window.electronAPI.deleteMapCache(newName);
    }

    // เซฟตามปกติ
    await saveEditedMapAsNew(newName);
    
    // สั่ง Sync เพื่อให้ Gallery อัปเดตรูปใหม่ (เผื่อรูป Thumbnail เปลี่ยน)
    setTimeout(() => {
        window.electronAPI.syncMaps();
    }, 1000); // รอสักนิดให้ไฟล์เขียนลง Disk เสร็จ
  });


  window.electronAPI.onSyncComplete((mapList) => {
    loadLocalMapsToGallery();
  });
  }

async function getExistingMapNames() {
    const maps = await window.electronAPI.getLocalMaps();
    return maps.map(m => m.name); // คืนค่าเป็น Array ของชื่อ ['map1', 'office1', ...]
}

function toggleMode(newMode) {
    if (mode === newMode) {
      cancelMode();
    } else {
      cancelMode();
      mode = newMode;
      const wallBtn = document.getElementById('toggle-wall-btn');
      const eraserBtn = document.getElementById('toggle-eraser-btn');
      
    if (mode === 'wall') {
        interactionCanvas.style.cursor = 'crosshair'; 
        if(wallBtn) {
            wallBtn.textContent = 'Wall :ON'; // เปลี่ยนข้อความ
            wallBtn.classList.add('active');  // เปลี่ยนสีปุ่ม
        }
    } else if (mode === 'eraser') {
        interactionCanvas.style.cursor = 'crosshair';
        if(eraserBtn) {
            eraserBtn.textContent = 'Eraser:ON'; // เปลี่ยนข้อความ
            eraserBtn.classList.add('active');    // เปลี่ยนสีปุ่ม
        }
    } else if (mode === 'none') {
        //ถ้าไม่มีโหมด (เช่นสั่ง toggleMode('none')) ต้องเป็นลูกศรปกติ
        interactionCanvas.style.cursor = 'default';
    } else {
        //โหมดเครื่องมืออื่นๆ (Goal, Pose, Draw) ใช้เป้าเล็งเพื่อความแม่นยำ
        interactionCanvas.style.cursor = 'crosshair';
    }
    
    // Update Button States
    if (newMode === 'goal') document.getElementById('set-goal-btn').classList.add('active');
    if (newMode === 'pose') document.getElementById('set-pose-btn').classList.add('active');
    if (newMode === 'draw') {
        const btn = document.getElementById('toggle-draw-mode');
        btn.textContent = 'Draw:ON';
        btn.classList.add('active');
        patrolPath.length = 0;
    }
    renderAllLayers();
  }
}

async function activateMap(mapName, meta) {
  let inflatedImageData;
  const cachedData = await window.electronAPI.loadMapCache(mapName);

  if (cachedData) {
      // Load Cache
      const finalImage = new Image();
      finalImage.src = cachedData.croppedImageBase64;
      await new Promise(resolve => finalImage.onload = resolve);
      const pixelData = base64ToUint8Array(cachedData.inflatedImageData.data);
      inflatedImageData = new ImageData(pixelData, cachedData.inflatedImageData.width, cachedData.inflatedImageData.height);

      activeMap.name = mapName;
      activeMap.base64 = finalImage.src;
      activeMap.meta = cachedData.newMeta;
      collisionMapData = new Uint32Array(inflatedImageData.data.buffer);
      collisionWidth = inflatedImageData.width;
      collisionHeight = inflatedImageData.height;
  } else {
      // Process New
      const { croppedImage, newMeta } = await autoCropMapImage(mapImage, meta);
      inflatedImageData = preprocessMapData(croppedImage);

      collisionMapData = new Uint32Array(inflatedImageData.data.buffer);
      collisionWidth = inflatedImageData.width;
      collisionHeight = inflatedImageData.height;
      
      activeMap.name = mapName;
      activeMap.base64 = croppedImage.src;
      activeMap.meta = newMeta;

      const dataToCache = {
          croppedImageBase64: activeMap.base64,
          newMeta: activeMap.meta,
          inflatedImageData: {
              width: inflatedImageData.width,
              height: inflatedImageData.height,
              data: bufferToBase64(inflatedImageData.data.buffer)
          }  
      };

      await window.electronAPI.saveMapCache(mapName, dataToCache);
  }

  document.getElementById('active-map-name').textContent = activeMap.name;
  localStorage.setItem('lastActiveMapName', mapName);

  // Prepare Hit Canvas & Dimmer
  mapHitCanvas = document.createElement('canvas');
  mapHitCanvas.width = inflatedImageData.width;
  mapHitCanvas.height = inflatedImageData.height;
  mapHitCtx = mapHitCanvas.getContext('2d', { willReadFrequently: true });
  mapHitCtx.putImageData(inflatedImageData, 0, 0);
  createDimmerMask(inflatedImageData);

  await updateHomePose();

  const finalMapImage = new Image();
  finalMapImage.onload = () => {
      mapImage = finalMapImage;
      resetStaticMapView(); 
      updateMapToolsState(true);
  };
  finalMapImage.src = activeMap.base64;
}

function isClickInsideBounds(worldPoint) {
  // เพิ่มการตรวจสอบว่า worldPoint ไม่ใช่ null หรือ undefined
  if (!activeMap.meta || !mapImage || !worldPoint) return false;

  const { origin, resolution } = activeMap.meta;
  const mapWidthInMeters = mapImage.width * resolution;
  const mapHeightInMeters = mapImage.height * resolution;

  const minX = origin[0];
  const maxX = origin[0] + mapWidthInMeters;
  const minY = origin[1];
  const maxY = origin[1] + mapHeightInMeters;

  // ตรวจสอบว่า worldPoint ที่คำนวณมาแล้ว อยู่ในขอบเขตหรือไม่
  if (worldPoint.x >= minX && worldPoint.x <= maxX &&
      worldPoint.y >= minY && worldPoint.y <= maxY) {
    return true;
  }

  return false;
}

function preprocessMapData(sourceImage) {
  console.log("StaticMap: Pre-processing with Uint32 Optimization...");
  
  // 1. เตรียม Canvas ชั่วคราว
  const tempCanvas = document.createElement('canvas');
  const tempCtx = tempCanvas.getContext('2d');
  const width = sourceImage.width;
  const height = sourceImage.height;
  tempCanvas.width = width;
  tempCanvas.height = height;
  tempCtx.drawImage(sourceImage, 0, 0);

  // 2. ดึงข้อมูลมาเป็น 32-bit Integers
  const imageData = tempCtx.getImageData(0, 0, width, height);
  const originalData32 = new Uint32Array(imageData.data.buffer);
  
  // สร้าง Buffer ใหม่สำหรับผลลัพธ์
  const inflatedImageData = tempCtx.createImageData(width, height);
  const inflatedData32 = new Uint32Array(inflatedImageData.data.buffer);

  // ค่าสีในระบบ Little Endian (ABGR)
  // ROS Map: Occupied = 0 (Black), Free = 255 (White), Unknown = 128/205 (Gray)
  // ดังนั้น Black ใน Uint32 คือ 0xFF000000 (Full Alpha, B=0, G=0, R=0)
  
  // กำหนด Threshold: ถ้าค่าสีน้อยกว่านี้ถือเป็นสิ่งกีดขวาง
  // เราเช็คแค่ Byte แรก (สีแดง) ก็พอ: pixel & 0xFF
  const obstacleThreshold = 50; 
  const margin = 1; // ขยายขอบ 1 พิกเซล (รวมเป็น 3x3)

  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const i = y * width + x;
      let isNearObstacle = false;
      
      // --- FAST CHECK ---
      // ถ้าตัวมันเองเป็นสิ่งกีดขวางอยู่แล้ว ก็ไม่ต้องเช็คเพื่อนบ้าน
      const centerPixel = originalData32[i];
      if ((centerPixel & 0xFF) < obstacleThreshold) { 
          inflatedData32[i] = 0xFF000000; // สีดำทึบ (ABGR)
          continue;
      }

      // --- INFLATION LOOP (Optimized) ---
      // เช็คเพื่อนบ้านเฉพาะตอนจำเป็น
      checkNeighbor:
      for (let dy = -margin; dy <= margin; dy++) {
        for (let dx = -margin; dx <= margin; dx++) {
          if (dx === 0 && dy === 0) continue;

          const nx = x + dx;
          const ny = y + dy;

          if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
            const ni = ny * width + nx;
            const neighborPixel = originalData32[ni];
            
            // เช็คแค่ Channel สีแดง (Byte สุดท้าย) ว่าดำไหม
            if ((neighborPixel & 0xFF) < obstacleThreshold) {
              isNearObstacle = true;
              break checkNeighbor; // เจอแค่อันเดียวก็พอ หยุดลูปทันที (ประหยัดเวลา)
            }
          }
        }
      }

      if (isNearObstacle) {
        inflatedData32[i] = 0xFF000000; // ถมดำ (สิ่งกีดขวางที่ขยายแล้ว)
      } else {
        inflatedData32[i] = originalData32[i]; // สีเดิม (พื้นที่ว่าง/Unknown)
      }
    }
  }

  return inflatedImageData;
}

function setupCanvasEvents() {
  const canvas = interactionCanvas;
  canvas.addEventListener('mousedown', (e) => {
  if (mode === 'wall' || mode === 'eraser') {
        isDrawing = true;
        
        // เริ่มเส้นใหม่ ระบุประเภทตามโหมดปัจจุบัน
        mapEdits.push({ 
            type: mode, 
            points: [] 
        });
        
        addEditPoint(e); // เก็บจุดแรก
        renderObjects();
        return; 
        
    }
  if (e.button === 2) {
        mapView.handleMouseDown(e);
        return;
    }
  if (mode === 'draw' || mode === 'goal' || mode === 'pose') {
    const worldPoint = getWorldCoordsFromEvent(e);
    if (!isClickInsideBounds(worldPoint)) return;

    if (mode === 'draw') {
      if (patrolPath.length > 0) {
        const lastPoint = patrolPath[patrolPath.length - 1];
        
        // ถ้าพยายามปิด Loop (คลิกจุดแรก)
        if (isHoveringFirstPoint && patrolPath.length > 1) {
             if (isPathBlocked(lastPoint, patrolPath[0])) {
              patrolPath.length = 0;
              alert("Cannot close loop: Path crosses an obstacle!");
              renderObjects();
              return;
             }
             patrolPath.push({ ...patrolPath[0] });
             renderObjects();
             cancelMode();
             return;
        } 
        
        // ถ้าลากจุดใหม่
        if (isPathBlocked(lastPoint, worldPoint)) {
             // สั่นหน้าจอหรือแจ้งเตือนเล็กน้อย (Optional)
             console.log("Blocked!"); 
             return; //ห้ามวางจุดนี้
        }
      }
      // ถ้าผ่านฉลุย ให้วางจุดได้
      patrolPath.push(worldPoint);
      renderObjects();
    } else if (mode === 'goal') {
        isSettingGoal = true;
        poseStartPosition = worldPoint; // เก็บจุดเริ่มต้น
        renderObjects();
    } else if (mode === 'pose') { 
      isSettingPose = true;
      poseStartPosition = worldPoint; // ใช้ worldPoint ที่คำนวณไว้แล้ว
      renderObjects();
    }

  } else {
    mapView.handleMouseDown(e);
  }
});

  canvas.addEventListener('mouseup', (e) => {
    if ((mode === 'pose' && isSettingPose) || (mode === 'goal' && isSettingGoal)) {
      const endPoint = getWorldCoordsFromEvent(e);
      const dx = endPoint.x - poseStartPosition.x;
      const dy = endPoint.y - poseStartPosition.y;
      const yaw = (dx === 0 && dy === 0) ? 0 : Math.atan2(dy, dx);
      const quaternion = yawToQuaternion(yaw);
      
      const poseData = { position: poseStartPosition, orientation: quaternion };
      console.log(`Iniial pose :${poseData.position} orien : ${poseData.orientation}`)
      if (mode === 'pose') {
          window.electronAPI.setInitialPose(poseData);
          console.log
      } else {
          stopPatrol();
          const goalData = {
          header: { 
            frame_id: 'map', 
            stamp: { secs: 0, nsecs: 0 } 
          },
          pose: {
            position: { 
                x: poseStartPosition.x, 
                y: poseStartPosition.y, 
                z: 0.0
            },
            orientation: quaternion
        }
    };
          setGoalPoint(goalData.pose);
          const shouldLoop = patrolState.isLooping;
          window.electronAPI.startPatrol([goalData], shouldLoop); 
          console.log("New Goal Set via UI");
          
      }
      isSettingPose = false;
      isSettingGoal = false;
      poseStartPosition = null;
      cancelMode();
    }
    isDrawing = false;
    mapView.handleMouseUp(e);
  });

  canvas.addEventListener('mouseleave', (e) => {
    isDrawing = false;
    mapView.handleMouseUp(e);
    if (isHoveringFirstPoint) {
      isHoveringFirstPoint = false;
      renderObjects();
    }
  });

  canvas.addEventListener('mousemove', (e) => {
  // อัปเดตตำแหน่งเมาส์ปัจจุบันเสมอ
  const rect = canvas.getBoundingClientRect();
  currentMousePos.x = e.clientX - rect.left;
  currentMousePos.y = e.clientY - rect.top;

  if (e.buttons === 2) {
    mapView.handleMouseMove(e);
    return;
  }

  if ((mode === 'wall' || mode === 'eraser') && isDrawing) {
         addEditPoint(e);
         renderObjects();
     }

  if (mode === 'draw') {
    if (isDrawing) {
      addPathPoint(e);
      renderObjects();
    } else if (patrolPath.length > 0 && activeMap.meta) {
      const snapRadius = 10 / mapView.viewState.scale;
      const firstPoint = patrolPath[0];
      const { resolution, origin } = activeMap.meta;
      
      // 1. แปลงพิกัด "จุดแรก" (World) ให้เป็น "พิกัดพิกเซลบนแผนที่" (Map Pixel) ด้วยสูตรดั้งเดิม
      const firstPointPx = (firstPoint.x - origin[0]) / resolution;
      const firstPointPy = mapImage.height - ((firstPoint.y - origin[1]) / resolution);

      // 2. แปลงพิกัด "เมาส์" (Screen) ให้เป็น "พิกัดพิกเซลบนแผนที่" (Map Pixel)
      const mousePx = (currentMousePos.x - mapView.viewState.offsetX) / mapView.viewState.scale;
      const mousePy = (currentMousePos.y - mapView.viewState.offsetY) / mapView.viewState.scale;

      // 3. คำนวณระยะห่างในระบบพิกัดเดียวกัน
      const distance = Math.sqrt(Math.pow(mousePx - firstPointPx, 2) + Math.pow(mousePy - firstPointPy, 2));
      
      const previouslyHovering = isHoveringFirstPoint;
      isHoveringFirstPoint = distance < snapRadius;

      if (previouslyHovering !== isHoveringFirstPoint) {
          canvas.style.cursor = isHoveringFirstPoint ? 'pointer' : 'crosshair';
          renderObjects();
      }
    }
  } 
  else if (mode === 'pose') {
    if (isSettingPose) {
      renderObjects();
    }
  }else if (mode === 'goal' && isSettingGoal) {
        renderObjects();
  }
   else {
    mapView.handleMouseMove(e);
  }
  });


  window.addEventListener('resize', () => {
    if(canvas.classList.contains('hidden')) return;
    resizeAllCanvases();
    resetStaticMapView();
  });

  canvas.addEventListener('contextmenu', (e) => {
    if (mode !== 'none') {
      e.preventDefault();
    }
  });
}

function addEditPoint(e) {
    const worldPoint = getWorldCoordsFromEvent(e);
    if (worldPoint && isClickInsideBounds(worldPoint)) {
        // ใส่จุดลงใน edit ล่าสุด
        if (mapEdits.length > 0) {
            mapEdits[mapEdits.length - 1].points.push(worldPoint);
        }
    }
}

export function undoLastEdit() {
    if (mapEdits.length > 0) {
        mapEdits.pop(); // ลบการกระทำล่าสุด
        renderObjects(); // วาดใหม่
        console.log("Undo successful");
    }
}

function drawUserWalls(ctx) {
    if (mapEdits.length === 0 || !activeMap?.meta || !mapImage) return;
    const { resolution, origin } = activeMap.meta;
    const imgH = mapImage.height;

    ctx.save();
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    
    // ความหนาเส้น (ปรับตามใจชอบ)
    // การหารด้วย scale ถูกต้องแล้ว เพื่อให้เส้นไม่บวมเวลาซูม
    ctx.lineWidth = 10 / mapView.viewState.scale; 

    mapEdits.forEach(edit => {
        if (edit.points.length < 2) return;
        
        //เลือกสีตามประเภท
        if (edit.type === 'wall') {
            ctx.strokeStyle = '#000000'; // สีดำ
        } else if (edit.type === 'eraser') {
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.7)'; // สีขาว
        }

        ctx.beginPath();
        edit.points.forEach((point, i) => {
            const px = (point.x - origin[0]) / resolution;
            const py = imgH - ((point.y - origin[1]) / resolution);
            if (i === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
        });
        ctx.stroke();
    });
    ctx.restore();
}

export async function saveEditedMapAsNew(newName) {
    if (!activeMap.name) return;

    // 1. โหลดข้อมูลแผนที่ "ต้นฉบับ" (ตัวเต็ม ไม่ใช่ตัว Crop ที่โชว์อยู่)
    // เราต้องดึงใหม่จาก Backend เพื่อความชัวร์ว่าเป็นไฟล์ Original จริงๆ
    const originalMapData = await window.electronAPI.getMapDataByName(activeMap.name);
    
    if (!originalMapData.success) {
        console.log("❌ Failed to load original map base.");
        return;
    }

    // 2. สร้าง Image Object จากรูปต้นฉบับ
    const fullImage = new Image();
    fullImage.src = originalMapData.base64;
    await new Promise(resolve => fullImage.onload = resolve);

    // 3. เตรียม Canvas ขนาดเท่า "รูปต้นฉบับ"
    const tempCanvas = document.createElement('canvas');
    tempCanvas.width = fullImage.width;
    tempCanvas.height = fullImage.height;
    const tCtx = tempCanvas.getContext('2d');

    // 4. วาดรูปต้นฉบับลงไป
    tCtx.imageSmoothingEnabled = false;
    tCtx.drawImage(fullImage, 0, 0);

    // 5. วาดเส้น Edits ทับลงไป
    // ⚠️ สำคัญ: ต้องใช้ Meta ของ "ต้นฉบับ" ในการคำนวณพิกัด
    const { resolution, origin } = originalMapData.meta; 
    
    tCtx.lineCap = 'round';
    tCtx.lineJoin = 'round';
    // ปรับความหนาเส้น (เนื่องจากรูปเต็มอาจจะใหญ่กว่ารูป Crop ต้องกะขนาดให้ดี)
    // หรือใช้สูตร: 15 / scale ถ้าต้องการ
    tCtx.lineWidth = 10 / mapView.viewState.scale; ; 

    mapEdits.forEach(edit => {
        if (edit.points.length < 2) return;
        
        if (edit.type === 'wall') {
            tCtx.strokeStyle = '#000000'; 
        } else {
            tCtx.strokeStyle = '#FFFFFF'; 
        }

        tCtx.beginPath();
        edit.points.forEach((point, i) => {
            // ⚠️ สูตรคำนวณ: ใช้ origin ของต้นฉบับ (originalMapData.meta.origin)
            // point.x/y คือ World Coordinate (เมตร) ซึ่งเป็นค่าสากล ไม่เปลี่ยนตามการ Crop
            const px = (point.x - origin[0]) / resolution;
            const py = fullImage.height - ((point.y - origin[1]) / resolution);
            
            if (i === 0) tCtx.moveTo(px, py);
            else tCtx.lineTo(px, py);
        });
        tCtx.stroke();
    });

    // 6. แปลงเป็น Base64
    const newBase64 = tempCanvas.toDataURL('image/png');

    // 7. สร้าง YAML Content โดยใช้ค่า Origin/Resolution ของ "ต้นฉบับ"
    // เพราะเราเซฟรูปขนาดเท่าเดิม Origin ก็ต้องเท่าเดิม
    const x = origin[0];
    const y = origin[1];
    const th = origin[2] || 0.0;

    const newYamlContent = `image: ${newName}.pgm
resolution: ${resolution}
origin: [${x}, ${y}, ${th}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
`;

    console.log(`Saving full-size map: ${newName}`);

    // 8. ส่งไปบันทึก
    const result = await window.electronAPI.saveEditedMap(newName, newBase64, newYamlContent);

    if (result.success) {
        alert("Map saved successfully (Full Size)!");
        // ... (Logic การเคลียร์ Cache หรือ Reload Gallery เดิมของคุณ) ...
        if (newName === activeMap.name) {
             await window.electronAPI.deleteMapCache(newName);
        }
        window.electronAPI.syncMaps();
        toggleMode('none'); 
        mapEdits.length = 0; // เคลียร์ Edits หลังบันทึก
    } else {
        alert("❌ Failed to save map: " + result.message);
    }
}

export function cancelMode() {
  mode = 'none';
  isDrawing = false;
  isHoveringFirstPoint = false;
  isSettingPose = false;
  isSettingGoal = false;
  poseStartPosition = null;
  
  if (interactionCanvas) interactionCanvas.style.cursor = 'grab';

  // รีเซ็ตปุ่ม UI ต่างๆ
  const goalBtn = document.getElementById('set-goal-btn');
  if(goalBtn) goalBtn.classList.remove('active');
  
  const poseBtn = document.getElementById('set-pose-btn');
  if(poseBtn) poseBtn.classList.remove('active');

  const drawModeBtn = document.getElementById('toggle-draw-mode');
  if(drawModeBtn) {
      drawModeBtn.textContent = 'Draw :OFF';
      drawModeBtn.classList.remove('active');
  }
  const wallBtn = document.getElementById('toggle-wall-btn');
  if(wallBtn) {
      wallBtn.textContent = 'Wall:OFF'; // คืนค่าเดิม
      wallBtn.classList.remove('active');
  }
  const eraserBtn = document.getElementById('toggle-eraser-btn');
  if(eraserBtn) {
      eraserBtn.textContent = 'Eraser:OFF'; // คืนค่าเดิม
      eraserBtn.classList.remove('active');
  }
  renderObjects();
}

function addPathPoint(e) {
  const worldPoint = getWorldCoordsFromEvent(e);
  if (worldPoint && isClickInsideBounds(e.clientX, e.clientY)) {
    patrolPath.push(worldPoint);
  }
}

function loadLocalMapsToGallery() {
  window.electronAPI.getLocalMaps().then((maps) => {
    const gallery = document.getElementById('map-gallery');
    gallery.innerHTML = '';
    maps.forEach(({ name, base64 }) => addMapToGallery(name, base64));
  });
}

async function autoCropMapImage(sourceImage, meta) {
  console.log("✂️ Cropping map to fit content...");
  const tempCanvas = document.createElement('canvas');
  const tempCtx = tempCanvas.getContext('2d');
  const width = sourceImage.width;
  const height = sourceImage.height;
  tempCanvas.width = width;
  tempCanvas.height = height;
  tempCtx.drawImage(sourceImage, 0, 0);

  const imageData = tempCtx.getImageData(0, 0, width, height).data;
  const unknownColor = 205; // สีเทาของ ROS map

  let minX = width, minY = height, maxX = -1, maxY = -1;

  // 1. สแกนหาขอบเขตของแผนที่จริง (ที่ไม่ใช่สีเทา)
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const color = imageData[(y * width + x) * 4];
      if (color !== unknownColor) {
        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
      }
    }
  }

  if (maxX === -1) { // กรณีแผนที่ว่างเปล่า
    return { croppedImage: sourceImage, newMeta: meta };
  }
  const cropWidth = maxX - minX + 1;
  const cropHeight = maxY - minY + 1;
  const cropCanvas = document.createElement('canvas');
  cropCanvas.width = cropWidth;
  cropCanvas.height = cropHeight;
  const cropCtx = cropCanvas.getContext('2d');
  cropCtx.drawImage(sourceImage, minX, minY, cropWidth, cropHeight, 0, 0, cropWidth, cropHeight);

  const finalImage = new Image();
  finalImage.src = cropCanvas.toDataURL();
  await new Promise(resolve => finalImage.onload = resolve);

  const newMeta = JSON.parse(JSON.stringify(meta));
  newMeta.origin[0] = meta.origin[0] + minX * meta.resolution;
  newMeta.origin[1] = meta.origin[1] + (sourceImage.height - maxY - 1) * meta.resolution;

   return { croppedImage: finalImage, newMeta };
}

function addMapToGallery(name, base64) {
  const img = document.createElement('img');
  img.src = base64;
  img.alt = name;
  img.title = name;
  img.className = 'map-thumb';
  img.style.cursor = 'pointer';
  img.addEventListener('click', async () => {
    console.log(`👁️ Previewing map: ${name}`);
    // 1. โหลดข้อมูล Meta ชั่วคราวสำหรับ Preview
    const result = await window.electronAPI.getMapMeta(name);
    if (!result.success) {
      alert(`Could not load metadata for ${name}`);
      return;
    }
    // 2. เก็บข้อมูลที่เลือกลงในตัวแปรชั่วคราว
    current_map_select = { name, base64, meta: result.data };
    // 3. โหลดและแสดงรูปภาพต้นฉบับเพื่อ Preview
    mapImage = new Image();
    mapImage.onload = () => {
      resetStaticMapView(); 
    };
    mapImage.src = base64;
  });
  document.getElementById('map-gallery').appendChild(img);
}

function createDimmerMask(imageData) {
  if (!imageData) return;
  const maskCanvas = document.createElement('canvas');
  const maskCtx = maskCanvas.getContext('2d', { willReadFrequently: true });
  maskCanvas.width = imageData.width;
  maskCanvas.height = imageData.height;
  
  // สร้างข้อมูลภาพใหม่เพื่อไม่ให้กระทบต้นฉบับ
  const maskImageData = new ImageData(
    new Uint8ClampedArray(imageData.data),
    imageData.width,
    imageData.height
  );
  const data = maskImageData.data;
  const freeSpaceThreshold = 250;

  for (let i = 0; i < data.length; i += 4) {
    const colorValue = data[i];
    if (colorValue > freeSpaceThreshold) {
      data[i + 3] = 0; // โปร่งใส
    } else {
      data.set([0, 0, 0, 150], i); // สีดำโปร่งแสง
    }
  }
  maskCtx.putImageData(maskImageData, 0, 0);
  dimmerMaskImage = new Image();
  dimmerMaskImage.src = maskCanvas.toDataURL();
  console.log("🎨 StaticMap: Pixel-perfect dimmer mask created from inflated map.");
}

function bufferToBase64(buffer) {
    let binary = '';
    const bytes = new Uint8Array(buffer);
    const len = bytes.byteLength;
    for (let i = 0; i < len; i++) {
        binary += String.fromCharCode(bytes[i]);
    }
    return window.btoa(binary);
}

function base64ToUint8Array(base64) {
    const binary_string = window.atob(base64);
    const len = binary_string.length;
    const bytes = new Uint8ClampedArray(len);
    for (let i = 0; i < len; i++) {
        bytes[i] = binary_string.charCodeAt(i);
    }
    return bytes;
}

function getWorldCoordsFromEvent(e) {
  if (!activeMap.meta || !mapImage) return null;
  const rect = interactionCanvas.getBoundingClientRect();
  const clickX = e.clientX - rect.left;
  const clickY = e.clientY - rect.top;

  const px = (clickX - mapView.viewState.offsetX) / mapView.viewState.scale;
  const py = (clickY - mapView.viewState.offsetY) / mapView.viewState.scale;
  
  // --- ✅ กลับมาใช้สูตรดั้งเดิมที่ถูกต้อง ---
  return {
    x: activeMap.meta.origin[0] + (px * activeMap.meta.resolution),
    y: activeMap.meta.origin[1] + ((mapImage.height - py) * activeMap.meta.resolution)
  };
}

function drawLaserScan(ctx) {
  if (!latestScan || !robotPose.position || !activeMap?.meta || !mapImage) return;

  const { resolution, origin } = activeMap.meta;
  const mapImgHeight = mapImage.height;
  const robotYaw = getYawFromQuaternion(robotPose.orientation);

  ctx.fillStyle = 'rgba(255, 0, 255, 0.7)'; // สีชมพูโปร่งแสง

  // ใช้ for loop และ fillRect เพื่อประสิทธิภาพสูงสุด
  for (let i = 0; i < latestScan.ranges.length; i++) {
    const range = latestScan.ranges[i];
    
    if (range < 0.1 || range > 10.0) continue; 

    const angle = latestScan.angle_min + i * latestScan.angle_increment;
    const totalAngle = robotYaw + angle;
    
    const worldX = robotPose.position.x + range * Math.cos(totalAngle);
    const worldY = robotPose.position.y + range * Math.sin(totalAngle);

    const px = (worldX - origin[0]) / resolution;
    const py = mapImgHeight - ((worldY - origin[1]) / resolution);
    
    // วาดสี่เหลี่ยมเล็กๆ ขนาด 2x2 pixels (ในพิกัดแผนที่)
    // ขนาดจะถูกปรับตามการซูมโดยอัตโนมัติ
    ctx.fillRect(px, py, 2 / mapView.viewState.scale, 2 / mapView.viewState.scale);
  }
}

function drawRobot(ctx) {
  if (!robotPose?.position || !activeMap?.meta || !mapImage) return;

  const { resolution, origin } = activeMap.meta;
  const mapImgHeight = mapImage.height;

  // แปลง World Coordinate เป็น Map Pixel Coordinate
  const px = (robotPose.position.x - origin[0]) / resolution;
  const py = mapImgHeight - ((robotPose.position.y - origin[1]) / resolution);
  const yaw = getYawFromQuaternion(robotPose.orientation);

  ctx.save();
  ctx.translate(px, py); // ย้ายจุดศูนย์กลางไปที่ตำแหน่งหุ่นยนต์
  ctx.rotate(-yaw);     // หมุน Canvas ตามทิศทางหุ่นยนต์

  // วาดรูปสามเหลี่ยมแทนตัวหุ่นยนต์
  const scale = 1.0 / mapView.viewState.scale; // ทำให้ขนาดหุ่นยนต์คงที่เมื่อซูม
  ctx.beginPath();
  ctx.moveTo(10 * scale, 0);
  ctx.lineTo(-5 * scale, -5 * scale);
  ctx.lineTo(-5 * scale, 5 * scale);
  ctx.closePath();
  ctx.fillStyle = 'rgba(255, 0, 0, 0.8)'; // สีแดง
  ctx.fill();

  ctx.strokeStyle = '#000000';
  ctx.lineWidth = 1;
  ctx.stroke();
  
  ctx.restore();
}

function drawPatrolPath(ctx) {
    // ถ้าไม่มีเส้นทาง, ข้อมูล meta, หรือรูปแผนที่ ให้หยุดทำงาน
    if (patrolState.patrolPath.length < 1 || !activeMap?.meta || !mapImage) return;

    const { resolution, origin } = activeMap.meta;
    const mapImgHeight = mapImage.height;

    // --- 1. วาดเส้นเชื่อมระหว่างจุด ---
    if (patrolState.patrolPath.length > 1) {
        ctx.strokeStyle = 'orange';
        ctx.lineWidth = 2 / mapView.viewState.scale; // ทำให้เส้นหนาเท่าเดิมไม่ว่าจะซูมแค่ไหน
        ctx.setLineDash([5, 5]); // ทำให้เป็นเส้นประ
        ctx.beginPath();
        patrolState.patrolPath.forEach((point, i) => {
            const px = (point.x - origin[0]) / resolution;
            const py = mapImgHeight - ((point.y - origin[1]) / resolution);
            if (i === 0) {
                ctx.moveTo(px, py);
            } else {
                ctx.lineTo(px, py);
            }
        });
        ctx.stroke();
        ctx.setLineDash([]); // คืนค่าเป็นเส้นทึบ
    }

    // --- 2. วาดจุด Waypoint แต่ละจุด ---
    patrolState.patrolPath.forEach((point, i) => {
        const px = (point.x - origin[0]) / resolution;
        const py = mapImgHeight - ((point.y - origin[1]) / resolution);
        
        // ตรวจสอบว่ากำลัง hover ที่จุดแรกหรือไม่ เพื่อเปลี่ยนขนาดและสี
        const isHoveredStartPoint = (i === 0 && isHoveringFirstPoint);
        const radius = (isHoveredStartPoint ? 8 : 6) / mapView.viewState.scale;
        
        ctx.beginPath();
        ctx.arc(px, py, radius, 0, 2 * Math.PI);
        ctx.fillStyle = isHoveredStartPoint ? '#00FF00' : 'cyan'; // ถ้า hover เป็นสีเขียว, ปกติเป็นสีฟ้า
        ctx.fill();
    });
}

function drawInteractionUI(ctx) {
    if (!activeMap.meta) return;

    if (isSettingPose && poseStartPosition) {
        drawArrow(ctx, poseStartPosition, currentMousePos, 'rgba(0, 255, 0, 0.9)');
    }

    if (isSettingGoal && poseStartPosition) {
        drawArrow(ctx, poseStartPosition, currentMousePos, 'rgba(255, 0, 0, 0.9)');
    }

    if (mode === 'draw' && patrolPath.length > 0) {
        drawDashedLineToMouse(ctx);
    }
}

function drawDashedLineToMouse(ctx) {
    if (!activeMap.meta || !mapImage || patrolPath.length === 0) return;

    const lastPoint = patrolPath[patrolPath.length - 1];
    
    // แปลงเมาส์เป็น World Coord เพื่อเช็ค Collision
    const rect = interactionCanvas.getBoundingClientRect();
    const mousePx = (currentMousePos.x - mapView.viewState.offsetX) / mapView.viewState.scale;
    const mousePy = (currentMousePos.y - mapView.viewState.offsetY) / mapView.viewState.scale;
    
    const mouseWorld = {
        x: activeMap.meta.origin[0] + (mousePx * activeMap.meta.resolution),
        y: activeMap.meta.origin[1] + ((mapImage.height - mousePy) * activeMap.meta.resolution)
    };

    //เช็คว่าเส้นทางติดกำแพงไหม
    const isBlocked = isPathBlocked(lastPoint, mouseWorld);

    // การคำนวณ Screen Coordinates เดิม
    const { resolution, origin } = activeMap.meta;
    const mapImgHeight = mapImage.height;
    const lastPxMap = (lastPoint.x - origin[0]) / resolution;
    const lastPyMap = mapImgHeight - (lastPoint.y - origin[1]) / resolution;
    const lastScreenX = lastPxMap * mapView.viewState.scale + mapView.viewState.offsetX;
    const lastScreenY = lastPyMap * mapView.viewState.scale + mapView.viewState.offsetY;

    ctx.save();
    
    if (isBlocked) {
        ctx.strokeStyle = 'rgba(255, 0, 0, 0.9)'; // สีแดงเข้ม
        ctx.lineWidth = 3;
        // เปลี่ยน cursor เป็นห้ามผ่านทันที
        interactionCanvas.style.cursor = 'not-allowed'; 
    } else {
        ctx.strokeStyle = 'rgba(0, 255, 255, 0.7)'; // สีฟ้าปกติ
        ctx.lineWidth = 2;
        // เปลี่ยน cursor เป็นเป้าเล็งปกติ
        interactionCanvas.style.cursor = 'crosshair';
    }

    ctx.setLineDash([5, 5]);
    ctx.beginPath();
    ctx.moveTo(lastScreenX, lastScreenY);
    ctx.lineTo(currentMousePos.x, currentMousePos.y);
    ctx.stroke();
    ctx.restore();
}

function drawGoal(ctx) {
    // ตรวจสอบว่ามีข้อมูล Goal Point และข้อมูลที่จำเป็นอื่นๆ ครบหรือไม่
    if (!goalPoint?.position || !activeMap?.meta || !mapImage) {
        return;
    }

    const { resolution, origin } = activeMap.meta;
    const mapImgHeight = mapImage.height;
    const { position, orientation } = goalPoint;

    // แปลง World Coordinates เป็น Map Pixel Coordinates
    const px = (position.x - origin[0]) / resolution;
    const py = mapImgHeight - ((position.y - origin[1]) / resolution);
    const scale = 1.0 / mapView.viewState.scale; // สเกลสำหรับวาดให้ขนาดคงที่

    // --- วาดจุดวงกลม ---
    ctx.beginPath();
    ctx.arc(px, py, 6 * scale, 0, 2 * Math.PI);
    ctx.fillStyle = 'red';
    ctx.fill();
    ctx.strokeStyle = 'white';
    ctx.lineWidth = 1 * scale;
    ctx.stroke();

    // --- วาดลูกศรแสดงทิศทาง ---
    if (orientation) {
        const yaw = getYawFromQuaternion(orientation);
        const arrowLength = 15 * scale;

        ctx.save();
        ctx.translate(px, py); // ย้ายจุดศูนย์กลางไปที่ Goal
        ctx.rotate(-yaw);      // หมุนตามทิศทาง
        
        ctx.beginPath();
        ctx.moveTo(arrowLength, 0);
        ctx.lineTo(arrowLength * 0.5, -5 * scale);
        ctx.lineTo(arrowLength * 0.5, 5 * scale);
        ctx.closePath();
        ctx.fillStyle = 'red';
        ctx.fill();
        
        ctx.restore();
    }
}

function drawArrow(ctx, startWorldPos, endScreenPos, color) {
    // ตรวจสอบข้อมูลที่จำเป็น
    if (!activeMap.meta || !mapImage || !startWorldPos || !endScreenPos) {
        return;
    }

    // --- 1. แปลงพิกัด "จุดเริ่มต้น" จาก World -> Screen ---
    const { resolution, origin } = activeMap.meta;
    const mapImgHeight = mapImage.height;
    
    // World -> Map Pixel
    const startPx = (startWorldPos.x - origin[0]) / resolution;
    const startPy = mapImgHeight - ((startWorldPos.y - origin[1]) / resolution);
    
    // Map Pixel -> Screen
    const startScreenX = startPx * mapView.viewState.scale + mapView.viewState.offsetX;
    const startScreenY = startPy * mapView.viewState.scale + mapView.viewState.offsetY;

    // --- 2. "จุดสิ้นสุด" เป็น Screen Coordinates อยู่แล้ว ---
    const endScreenX = endScreenPos.x;
    const endScreenY = endScreenPos.y;

    // --- 3. วาดเส้นและหัวลูกศร ---
    ctx.save(); // บันทึกสถานะ context
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = 2;

    // วาดเส้นตรง (ก้านลูกศร)
    ctx.beginPath();
    ctx.moveTo(startScreenX, startScreenY);
    ctx.lineTo(endScreenX, endScreenY);
    ctx.stroke();
    
    // วาดหัวลูกศร
    const angle = Math.atan2(endScreenY - startScreenY, endScreenX - startScreenX);
    const headlen = 10; // ขนาดของหัวลูกศร
    ctx.beginPath();
    ctx.moveTo(endScreenX, endScreenY);
    ctx.lineTo(endScreenX - headlen * Math.cos(angle - Math.PI / 6), endScreenY - headlen * Math.sin(angle - Math.PI / 6));
    ctx.lineTo(endScreenX - headlen * Math.cos(angle + Math.PI / 6), endScreenY - headlen * Math.sin(angle + Math.PI / 6));
    ctx.closePath();
    ctx.stroke();
    ctx.fill();

    ctx.restore(); // คืนค่า context
}

function isPathBlocked(p1, p2) {
  if (!collisionMapData || !activeMap.meta) return false;

  const { resolution, origin } = activeMap.meta;
  
  // แปลง World Coordinate -> Map Pixel Coordinate
  // (ใช้ Math.floor เพื่อให้ได้ index ที่แน่นอน)
  const x0 = Math.floor((p1.x - origin[0]) / resolution);
  const y0 = Math.floor(collisionHeight - ((p1.y - origin[1]) / resolution));
  const x1 = Math.floor((p2.x - origin[0]) / resolution);
  const y1 = Math.floor(collisionHeight - ((p2.y - origin[1]) / resolution));

  // Bresenham's Line Algorithm (เดินทีละพิกเซลบนเส้นตรง)
  let dx = Math.abs(x1 - x0);
  let dy = Math.abs(y1 - y0);
  let sx = (x0 < x1) ? 1 : -1;
  let sy = (y0 < y1) ? 1 : -1;
  let err = dx - dy;

  let x = x0;
  let y = y0;

  while (true) {
    // เช็คว่าพิกัดอยู่ในขอบเขตแผนที่ไหม
    if (x >= 0 && x < collisionWidth && y >= 0 && y < collisionHeight) {
      const index = y * collisionWidth + x;
      const pixel = collisionMapData[index];
      
      // เช็คค่าสี: ถ้าค่าสีแดง (R) น้อยกว่า 50 ถือเป็นสิ่งกีดขวาง
      // (อิงตาม Logic การสร้าง inflatedImageData)
      if ((pixel & 0xFF) <= 250) { 
        return true; //เจอกำแพง หรือ พื้นที่ Unknown
      }
    }

    if (x === x1 && y === y1) break;
    
    let e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x += sx; }
    if (e2 < dx) { err += dx; y += sy; }
  }

  return false; // ✅ ทางสะดวก
}

async function loadLastActiveMap() {
  const lastMapName = localStorage.getItem('lastActiveMapName');
  if (!lastMapName) return;

  console.log(`Loading last used map: ${lastMapName}`);
  
  // เรียก API ที่ main.js เพื่อขอดึงข้อมูลแผนที่โดยตรง (ไม่ต้องผ่าน Gallery)
  const result = await window.electronAPI.getMapDataByName(lastMapName);
  
  if (result.success) {
      // ตั้งค่าตัวแปร selection รอไว้
      current_map_select = { 
          name: result.name, 
          base64: result.base64, 
          meta: result.meta 
      };

      // แสดงชื่อแผนที่บนหน้าจอ
      document.getElementById('active-map-name').textContent = result.name;

      // โหลดรูปมาแสดงเป็น Preview (ยังไม่ Activate จนกว่าจะกดปุ่ม Start)
      mapImage = new Image();
      mapImage.onload = () => {
          activateMap(current_map_select.name, current_map_select.meta)
          resetStaticMapView();
          renderAllLayers(); 
      };
      mapImage.src = result.base64;
      
      console.log("Last map loaded for preview.");
  } else {
      console.warn("⚠️ Could not load last map:", result.message);
  }
}

function showPrompt(title, defaultValue = "") {
    return new Promise((resolve) => {
        const modal = document.getElementById('custom-prompt-modal');
        const titleEl = document.getElementById('modal-title');
        const inputEl = document.getElementById('modal-input');
        const confirmBtn = document.getElementById('modal-confirm-btn');
        const cancelBtn = document.getElementById('modal-cancel-btn');

        // ตั้งค่าข้อความ
        titleEl.textContent = title;
        inputEl.value = defaultValue;
        modal.classList.remove('hidden');
        inputEl.focus();

        // ฟังก์ชันเมื่อจบการทำงาน (Cleanup)
        const cleanup = () => {
            modal.classList.add('hidden');
            confirmBtn.removeEventListener('click', onConfirm);
            cancelBtn.removeEventListener('click', onCancel);
            inputEl.removeEventListener('keydown', onKey);
        };

        const onConfirm = () => {
            cleanup();
            resolve(inputEl.value); // ส่งค่ากลับ
        };

        const onCancel = () => {
            cleanup();
            resolve(null); // ส่งค่า null (เหมือนกด Cancel ใน prompt ปกติ)
        };

        const onKey = (e) => {
            if (e.key === 'Enter') onConfirm();
            if (e.key === 'Escape') onCancel();
        };

        // ผูก Event
        confirmBtn.addEventListener('click', onConfirm);
        cancelBtn.addEventListener('click', onCancel);
        inputEl.addEventListener('keydown', onKey);
    });
}

async function updateHomePose() {
    if (!activeMap.name) return;
    try {
        const result = await window.electronAPI.getMapHome(activeMap.name);
        if (result.success) {
            homePose = result.data;
            console.log("🏠 Home pose loaded:", homePose);
        } else {
            homePose = null; // ถ้าไม่มี Home ให้เคลียร์ทิ้ง
        }
        renderObjects(); // วาดใหม่ทันทีที่โหลดเสร็จ
    } catch (err) {
        console.error("Failed to load home pose:", err);
    }
}

function drawHome(ctx) {
    if (!homePose || !activeMap?.meta || !mapImage) return;

    const { resolution, origin } = activeMap.meta;
    const mapImgHeight = mapImage.height;

    // 1. แปลง World Coordinate เป็น Map Pixel
    const px = (homePose.x - origin[0]) / resolution;
    const py = mapImgHeight - ((homePose.y - origin[1]) / resolution);
    
    // 2. คำนวณ Scale เพื่อให้ไอคอนขนาดคงที่เวลาซูม
    const scale = 1.0 / mapView.viewState.scale; 
    const size = 12 * scale; // ขนาดบ้าน

    ctx.save();
    ctx.translate(px, py);

    // (Optional) ถ้าอยากให้บ้านหมุนตามทิศที่ตั้งไว้
    // const yaw = getYawFromQuaternion({ x: homePose.ox, y: homePose.oy, z: homePose.oz, w: homePose.ow });
    // ctx.rotate(-yaw); 

    // 3. วาดรูปบ้าน (House Icon)
    ctx.beginPath();
    // หลังคา
    ctx.moveTo(0, -size); 
    ctx.lineTo(size, -size * 0.3);
    ctx.lineTo(size * 0.8, -size * 0.3);
    // ตัวบ้าน
    ctx.lineTo(size * 0.8, size);
    ctx.lineTo(-size * 0.8, size);
    ctx.lineTo(-size * 0.8, -size * 0.3);
    ctx.lineTo(-size, -size * 0.3);
    ctx.closePath();

    // ลงสี
    ctx.fillStyle = '#007bff'; // สีน้ำเงิน
    ctx.fill();
    ctx.strokeStyle = 'white';
    ctx.lineWidth = 2 * scale;
    ctx.stroke();

    // วาดตัว H ตรงกลาง
    ctx.fillStyle = 'white';
    ctx.font = `bold ${10 * scale}px Arial`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText("H", 0, size * 0.4);

    ctx.restore();
}