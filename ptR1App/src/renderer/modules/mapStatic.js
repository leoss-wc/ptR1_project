// modules/mapStatic.js
import { patrolPath , setGoalPoint} from './patrolState.js';
import { activeMap } from './mapState.js';
import * as mapView from './mapView.js'; 
import { robotPose } from './robotState.js';
import { latestScan } from './laserScanState.js';

let canvas, ctx, mapImage;
let isDrawing = false;
let isHoveringFirstPoint = false;
let current_map_select = { name: null, base64: null ,meta:null};
let goalPoint = null;
let isSettingGoal = false;
let mode = 'none';

let isSettingPose = false; 
let poseStartPosition = null;
let currentMousePos = { x: 0, y: 0 }; //เก็บตำแหน่งเมาส์ล่าสุดบน Canvas

let mapHitCanvas, mapHitCtx; //ตัวแปรสำหรับ Canvas ที่ใช้ตรวจสอบการคลิกแบบ Pixel-perfect

let dimmerMaskImage = null;//ตัวแปรสำหรับเก็บภาพมาสก์ Dimmer ที่สร้างขึ้น

let tempInitialPose = null; 

export function initStaticMap() {
  canvas = document.getElementById('staticMapCanvas');
  ctx = canvas.getContext('2d');
  bindUI();
  setupCanvasEvents();
  loadLocalMapsToGallery();
}

//ฟังก์ชันสำหรับ Reset View โดยใช้หลัก "Fit and Center"
function resetStaticMapView() {
  if (!canvas || !mapImage) return;
  console.log("StaticMap: View reset to fit and center.");
  const zoomX = canvas.width / mapImage.width;
  const zoomY = canvas.height / mapImage.height;

  const newScale = Math.min(zoomX, zoomY) * 0.95;

  mapView.viewState.scale = newScale;
  mapView.viewState.offsetX = (canvas.width - mapImage.width * newScale) / 2;
  mapView.viewState.offsetY = (canvas.height - mapImage.height * newScale) / 2;
  
  renderCanvas();
}

document.getElementById('select-map-btn').addEventListener('click', async () => {
  // 🔧 แก้ไข: ฟังก์ชันนี้จะทำหน้าที่ "ยืนยัน" และอัปเดต activeMap
  if (!current_map_select.name || !mapImage) {
    alert("❗ Please select a map from the gallery first.");
    return;
  }
  
  console.log(`✅ Activating map: ${current_map_select.name}`);
  
  // --- เริ่มกระบวนการประมวลผลและ Caching ---
  let inflatedImageData;
  const mapName = current_map_select.name;

  const cachedData = await window.electronAPI.loadMapCache(mapName);
  if (cachedData) {
      console.log("🗺️ StaticMap: Cache hit! Using cached data for activation.");
      // 1. โหลดข้อมูลจาก Cache
      const finalImage = new Image();
      finalImage.src = cachedData.croppedImageBase64;
      await new Promise(resolve => finalImage.onload = resolve);
      
      const pixelData = base64ToUint8Array(cachedData.inflatedImageData.data);
      inflatedImageData = new ImageData(pixelData, cachedData.inflatedImageData.width, cachedData.inflatedImageData.height);

      // 2. อัปเดต ActiveMap ด้วยข้อมูลจาก Cache
      activeMap.name = mapName;
      activeMap.base64 = finalImage.src;
      activeMap.meta = cachedData.newMeta;
      
  } else {
      console.log("🗺️ StaticMap: Cache miss! Performing full processing for activation...");
      // 1. Crop รูปและปรับ Meta Data
      const { croppedImage, newMeta } = await autoCropMapImage(mapImage, current_map_select.meta);
      
      // 2. ประมวลผล Obstacle Inflation
      inflatedImageData = preprocessMapData(croppedImage);
      
      // 3. อัปเดต ActiveMap ด้วยข้อมูลที่ประมวลผลใหม่
      activeMap.name = mapName;
      activeMap.base64 = croppedImage.src;
      activeMap.meta = newMeta;

      // 4. บันทึกข้อมูลทั้งหมดลง Cache
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

  // อัปเดต UI และส่งคำสั่งไป ROS
  document.getElementById('active-map-name').textContent = activeMap.name;
  localStorage.setItem('activeMapName', activeMap.name);
  window.electronAPI.selectMap(activeMap.name);
  
  alert(`Active map has been set to "${activeMap.name}".`);

  // อัปเดตเครื่องมือวาดภาพด้วยข้อมูลล่าสุด
  mapHitCanvas = document.createElement('canvas');
  mapHitCanvas.width = inflatedImageData.width;
  mapHitCanvas.height = inflatedImageData.height;
  mapHitCtx = mapHitCanvas.getContext('2d', { willReadFrequently: true });
  mapHitCtx.putImageData(inflatedImageData, 0, 0);
  createDimmerMask(inflatedImageData);

  const finalMapImage = new Image();
  finalMapImage.onload = () => {
      // 1. อัปเดตตัวแปร mapImage ที่ใช้ในการวาด
      mapImage = finalMapImage;
      // 2. สั่ง Reset View และวาด Canvas ใหม่ทั้งหมด
      resetStaticMapView(); 
  };
  // 3. ใช้ข้อมูลรูปภาพที่ผ่านการ Crop แล้วจาก activeMap
  finalMapImage.src = activeMap.base64;
});

function bindUI() {
  document.getElementById('zoom-in').addEventListener('click', () => {
    if (mapImage) {
      mapView.viewState.scale *= 1.2;
      renderCanvas();
    }
  });
  document.getElementById('zoom-out').addEventListener('click', () => {
    if (mapImage) {
      mapView.viewState.scale /= 1.2;
      renderCanvas();
    }
  });
  document.getElementById('reset-static-view-btn').addEventListener('click', resetStaticMapView);

  document.getElementById('clear-path-btn').addEventListener('click', () => {
    patrolPath.length = 0;
    goalPoint = null;
    cancelMode();
    renderCanvas();
  });
  document.getElementById('sync-maps-btn').addEventListener('click', () => {
    window.electronAPI.syncMaps();
  });
  document.getElementById('set-goal-btn').addEventListener('click', () => {
    if (mode === 'goal') {
      cancelMode();
      return;
    }
    patrolPath.length = 0;
    cancelMode(); 
    mode = 'goal';
    canvas.style.cursor = 'crosshair';
    document.getElementById('set-goal-btn').classList.add('active');
    renderCanvas();
  });
  const drawModeBtn = document.getElementById('toggle-draw-mode');
  
  document.getElementById('set-pose-btn').addEventListener('click', () => {
    if (mode === 'pose') {
      cancelMode();
      return;
    }
    cancelMode();
    mode = 'pose';
    canvas.style.cursor = 'crosshair';
    document.getElementById('set-pose-btn').classList.add('active');
  });


  drawModeBtn.addEventListener('click', () => {
    if (mode === 'draw') {
      cancelMode();
      return;
    }
    goalPoint = null;
    cancelMode();
    mode = 'draw';
    canvas.style.cursor = 'crosshair';
    drawModeBtn.textContent = 'Draw :ON';
    drawModeBtn.classList.add('active');
    patrolPath.length = 0;
    renderCanvas();
  });
  window.electronAPI.onSyncComplete((mapList) => {
    const gallery = document.getElementById('map-gallery');
    gallery.innerHTML = '';
    mapList.forEach(({ name, base64 }) => addMapToGallery(name, base64));
    loadLocalMapsToGallery();
  });
}

// ในไฟล์ mapStatic.js
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
  console.log("🗺️ StaticMap: Pre-processing map to inflate obstacles and preserve unknown space...");
  const tempCanvas = document.createElement('canvas');
  const tempCtx = tempCanvas.getContext('2d');
  const width = sourceImage.width;
  const height = sourceImage.height;
  tempCanvas.width = width;
  tempCanvas.height = height;
  tempCtx.drawImage(sourceImage, 0, 0);

  const originalData = tempCtx.getImageData(0, 0, width, height).data;
  const inflatedImageData = tempCtx.createImageData(width, height);
  const inflatedDataArray = inflatedImageData.data;

  const obstacleThreshold = 10;
  const marginSize = 1;

  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const index = (y * width + x) * 4;
      let isNearObstacle = false;

      // ตรวจสอบพื้นที่รอบๆ เพื่อขยายขอบเขต (Inflation)
      for (let j = -marginSize; j <= marginSize; j++) {
        for (let i = -marginSize; i <= marginSize; i++) {
          const checkX = x + i;
          const checkY = y + j;
          if (checkX >= 0 && checkX < width && checkY >= 0 && checkY < height) {
            const neighborIndex = (checkY * width + checkX) * 4;
            if (originalData[neighborIndex] < obstacleThreshold) {
              isNearObstacle = true;
              break;
            }
          }
        }
        if (isNearObstacle) break;
      }

      if (isNearObstacle) {
        // ถ้าอยู่ใกล้สิ่งกีดขวาง ให้ตั้งค่าเป็นสีดำ
        inflatedDataArray.set([0, 0, 0, 255], index);
      } else {
        // ✨ ถ้าไม่อยู่ใกล้สิ่งกีดขวาง ให้ใช้สีเดิมจากแผนที่ต้นฉบับ
        // ซึ่งจะช่วยรักษาสีเทา (Unknown Space) เอาไว้
        const originalColor = originalData[index];
        inflatedDataArray.set([originalColor, originalColor, originalColor, 255], index);
      }
    }
  }
  console.log("🗺️ StaticMap: Map pre-processing complete.");
  return inflatedImageData;
}

// ฟังก์ชันสำหรับวาด Overlay Dimmer
function drawBoundaryMask() {
  if (!mapImage || !canvas) return;
  const { offsetX, offsetY, scale } = mapView.viewState;

  // ฟังก์ชันนี้จะถูกเรียกใช้ใน "Screen Space" (หลังจาก ctx.restore() แล้ว)
  ctx.save(); 
  
  // 1. เริ่มวาดสี่เหลี่ยมสีดำทึบเต็มหน้าจอ
  ctx.fillStyle = "rgba(0, 0, 0, 0.6)";
  ctx.beginPath();
  ctx.rect(0, 0, canvas.width, canvas.height);

  // 2. สร้าง Path สำหรับ "เจาะรู" ที่ถูกหมุนไปพร้อมกับแผนที่
  //    - ย้ายจุดศูนย์กลางไปที่มุมซ้ายบนของแผนที่บนจอ
  ctx.translate(offsetX, offsetY);
  //    - ขยายตามการซูม
  ctx.scale(scale, scale);
  //    - หมุนในทิศทางเดียวกันกับแผนที่
  ctx.rotate(-Math.PI / 2);
  ctx.translate(-mapImage.height, 0);
  
  ctx.moveTo(0, 0);
  ctx.lineTo(mapImage.width, 0);
  ctx.lineTo(mapImage.width, mapImage.height);
  ctx.lineTo(0, mapImage.height);
  ctx.closePath();

  ctx.restore(); 


  ctx.fill("evenodd");
}

function setupCanvasEvents() {
  canvas.addEventListener('mousedown', (e) => {
  if (mode === 'draw' || mode === 'goal' || mode === 'pose') {
    
    const worldPoint = getWorldCoordsFromEvent(e);
    if (!isClickInsideBounds(worldPoint)) return;
    if (mode === 'draw') {
      if (isHoveringFirstPoint && patrolPath.length > 1) { 
        patrolPath.push({ ...patrolPath[0] });
        renderCanvas();
        cancelMode(); 
      } else { 
        patrolPath.push(worldPoint); // ใช้ worldPoint ที่คำนวณไว้แล้ว
        renderCanvas(); 
      }
    } else if (mode === 'goal') {
        isSettingGoal = true;
        poseStartPosition = worldPoint; // เก็บจุดเริ่มต้น
        renderCanvas();
    } else if (mode === 'pose') {
      tempInitialPose = null; 
      isSettingPose = true;
      poseStartPosition = worldPoint; // ใช้ worldPoint ที่คำนวณไว้แล้ว
      renderCanvas();
    }
    // --- สิ้นสุดส่วนที่แก้ไข ---

  } else {
    mapView.handleMouseDown(e);
  }
});

  canvas.addEventListener('mouseup', (e) => {
    if (mode === 'pose' && isSettingPose) {
      const endPoint = getWorldCoordsFromEvent(e);
      const dx = endPoint.x - poseStartPosition.x;
      const dy = endPoint.y - poseStartPosition.y;
      const yaw = Math.atan2(dy, dx);
      const quaternion = yawToQuaternion(yaw);
      
      const poseData = {
        position: poseStartPosition,
        orientation: quaternion,
      };
      window.electronAPI.setInitialPose(poseData);

      console.log("Switching to AMCL pose subscriber for localization mode.");
      window.electronAPI.switchPoseSubscriber('amcl');
      
      isSettingPose = false;
      poseStartPosition = null;
      cancelMode();
    }
    if (mode === 'goal' && isSettingGoal) {
        const endPoint = getWorldCoordsFromEvent(e);
        const dx = endPoint.x - poseStartPosition.x;
        const dy = endPoint.y - poseStartPosition.y;

        // ถ้าไม่มีการลาก (คลิกเฉยๆ) ให้ใช้ทิศทางเริ่มต้น
        const yaw = (dx === 0 && dy === 0) ? 0 : Math.atan2(dy, dx);
        const quaternion = yawToQuaternion(yaw);

        const goalPose = {
            position: poseStartPosition,
            orientation: quaternion,
        };
        
        // อัปเดต State และส่งไป ROS
        setGoalPoint(goalPose);
        // ส่งข้อมูลเป็น pose object ทั้งหมด
        window.electronAPI.sendSingleGoal({ pose: goalPose });
        console.log("New goal point set:", goalPose);

        
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
      renderCanvas();
    }
  });

  canvas.addEventListener('mousemove', (e) => {
  // อัปเดตตำแหน่งเมาส์ปัจจุบันเสมอ
  const rect = canvas.getBoundingClientRect();
  currentMousePos.x = e.clientX - rect.left;
  currentMousePos.y = e.clientY - rect.top;

  if (mode === 'draw') {
    if (isDrawing) {
      addPathPoint(e);
      renderCanvas();
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
          renderCanvas();
      }
    }
  } 
  else if (mode === 'pose') {
    if (isSettingPose) {
      renderCanvas();
    }
  }else if (mode === 'goal' && isSettingGoal) {
        renderCanvas();
  }
   else {
    mapView.handleMouseMove(e);
  }
  });


  window.addEventListener('resize', () => {
    if(canvas.classList.contains('hidden')) return;
    resetStaticMapView();
  });

  canvas.addEventListener('contextmenu', (e) => {
    if (mode !== 'none') {
      e.preventDefault();
      cancelMode();
    }
  });
}

function cancelMode() {
  mode = 'none';
  isDrawing = false;
  isHoveringFirstPoint = false;
  isSettingPose = false;
  isSettingGoal = false;
  poseStartPosition = null;
  canvas.style.cursor = 'grab';
  document.getElementById('set-goal-btn').classList.remove('active');
  const drawModeBtn = document.getElementById('toggle-draw-mode');
  drawModeBtn.textContent = 'Draw :OFF';
  drawModeBtn.classList.remove('active');
  renderCanvas();
}

function addPathPoint(e) {
  const worldPoint = getWorldCoordsFromEvent(e);
  if (worldPoint && isClickInsideBounds(e.clientX, e.clientY)) {
    patrolPath.push(worldPoint);
  }
}

export function renderCanvas() {
  if (!canvas || !mapImage) return;

  // 1. เตรียม Canvas และล้างหน้าจอ
  canvas.width = canvas.clientWidth;
  canvas.height = canvas.clientHeight;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.imageSmoothingEnabled = false;

  // 2. บันทึกสถานะ และจัดการ Pan/Zoom
  ctx.save();
  ctx.translate(mapView.viewState.offsetX, mapView.viewState.offsetY);
  ctx.scale(mapView.viewState.scale, mapView.viewState.scale);

  // --- ไม่มีการหมุนมุมมอง (NO ROTATION) ---
  
  // 3. วาดแผนที่ และ Dimmer Mask (ส่วนในแผนที่)
  ctx.drawImage(mapImage, 0, 0, mapImage.width, mapImage.height);
  if (dimmerMaskImage && (mode === 'draw' || mode === 'goal')) {
    ctx.drawImage(dimmerMaskImage, 0, 0, mapImage.width, mapImage.height);
  }

  // 4. วาด Path, Goal, etc.
  if (activeMap.meta) {
    const { resolution, origin } = activeMap.meta;
    const mapImgHeight = mapImage.height;

    // --- วาดเส้น Path ---
    if (patrolPath.length > 1) {
      ctx.strokeStyle = 'orange';
      ctx.lineWidth = 2 / mapView.viewState.scale;
      ctx.beginPath();
      patrolPath.forEach((point, i) => {
        const px = (point.x - origin[0]) / resolution;
        const py = mapImgHeight - ((point.y - origin[1]) / resolution);
        i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
      });
      ctx.stroke();
    }
    
    // --- วาดจุดบน Path ---
    patrolPath.forEach((point, i) => {
        const px = (point.x - origin[0]) / resolution;
        const py = mapImgHeight - ((point.y - origin[1]) / resolution);
        const radius = (i === 0 && isHoveringFirstPoint ? 8 : 6) / mapView.viewState.scale;
        ctx.beginPath();
        ctx.arc(px, py, radius, 0, 2 * Math.PI);
        ctx.fillStyle = (i === 0 && isHoveringFirstPoint) ? '#00FF00' : 'cyan';
        ctx.fill();
    });

    // --- วาด Goal ---
    if (goalPoint) {
      const px = (goalPoint.x - origin[0]) / resolution;
      const py = mapImgHeight - ((goalPoint.y - origin[1]) / resolution);
      ctx.beginPath();
      ctx.arc(px, py, 6 / mapView.viewState.scale, 0, 2 * Math.PI);
      ctx.fillStyle = 'red';
      ctx.fill();
      ctx.lineWidth = 2 / mapView.viewState.scale;
      ctx.strokeStyle = 'white';
      ctx.stroke();
    }

    
    drawLaserScan();
    drawRobot();
    drawTempInitialPose();
  }

  // 5. คืนค่า Context จาก Pan/Zoom
  ctx.restore();

  // --- ณ จุดนี้ เรากลับมาวาดใน "พิกัดหน้าจอ" (Screen Space) ตามปกติ ---

  // 6. วาด Dimmer Mask (ส่วนนอกแผนที่)
  if (mode === 'draw' || mode === 'goal') {
    drawBoundaryMask();
  }

  if (mode === 'draw' && patrolPath.length > 0) {
    const lastPoint = patrolPath[patrolPath.length - 1];
    const { resolution, origin } = activeMap.meta;
    
    // แปลงจุดสุดท้าย (World) -> Screen Coords
    const lastPx = (lastPoint.x - origin[0]) / resolution;
    const lastPy = mapImage.height - (lastPoint.y - origin[1]) / resolution;
    const lastScreenX = lastPx * mapView.viewState.scale + mapView.viewState.offsetX;
    const lastScreenY = lastPy * mapView.viewState.scale + mapView.viewState.offsetY;

    // จุดสิ้นสุดคือตำแหน่งเมาส์ปัจจุบัน (ซึ่งเป็น Screen Coords อยู่แล้ว)
    const mouseScreenX = currentMousePos.x;
    const mouseScreenY = currentMousePos.y;

    // วาดเส้นประนำสายตา
    ctx.save();
    ctx.strokeStyle = 'rgba(0, 255, 255, 0.7)'; // สีฟ้าโปร่งแสง
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 5]); // ทำให้เป็นเส้นประ
    ctx.beginPath();
    ctx.moveTo(lastScreenX, lastScreenY);
    ctx.lineTo(mouseScreenX, mouseScreenY);
    ctx.stroke();
    ctx.restore();
  }
  
  // 7. วาดลูกศร Initial Pose
  if (isSettingPose && poseStartPosition) {
    const { resolution, origin } = activeMap.meta;
    
    // แปลงจุดเริ่มต้น (World) -> Map Pixel (ดั้งเดิม)
    const startPx = (poseStartPosition.x - origin[0]) / resolution;
    const startPy = mapImage.height - ((poseStartPosition.y - origin[1]) / resolution);
    
    // แปลง Map Pixel -> Screen Coords
    const startScreenX = startPx * mapView.viewState.scale + mapView.viewState.offsetX;
    const startScreenY = startPy * mapView.viewState.scale + mapView.viewState.offsetY;

    // จุดสิ้นสุดคือตำแหน่งเมาส์ปัจจุบัน
    const endScreenX = currentMousePos.x;
    const endScreenY = currentMousePos.y;

    // วาดเส้นและหัวลูกศร
    ctx.save();
    ctx.strokeStyle = 'rgba(0, 255, 0, 0.9)';
    ctx.fillStyle = 'rgba(0, 255, 0, 0.7)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(startScreenX, startScreenY);
    ctx.lineTo(endScreenX, endScreenY);
    ctx.stroke();
    
    const angle = Math.atan2(endScreenY - startScreenY, endScreenX - startScreenX);
    const headlen = 10;
    ctx.beginPath();
    ctx.moveTo(endScreenX, endScreenY);
    ctx.lineTo(endScreenX - headlen * Math.cos(angle - Math.PI / 6), endScreenY - headlen * Math.sin(angle - Math.PI / 6));
    ctx.lineTo(endScreenX - headlen * Math.cos(angle + Math.PI / 6), endScreenY - headlen * Math.sin(angle + Math.PI / 6));
    ctx.closePath();
    ctx.stroke();
    ctx.fill();
    ctx.restore();
  }

  if (isSettingGoal && poseStartPosition) {
    const { resolution, origin } = activeMap.meta;
    
    // แปลงจุดเริ่มต้น (World) -> Map Pixel (ดั้งเดิม)
    const startPx = (poseStartPosition.x - origin[0]) / resolution;
    const startPy = mapImage.height - ((poseStartPosition.y - origin[1]) / resolution);
    
    // แปลง Map Pixel -> Screen Coords
    const startScreenX = startPx * mapView.viewState.scale + mapView.viewState.offsetX;
    const startScreenY = startPy * mapView.viewState.scale + mapView.viewState.offsetY;

    // จุดสิ้นสุดคือตำแหน่งเมาส์ปัจจุบัน
    const endScreenX = currentMousePos.x;
    const endScreenY = currentMousePos.y;

    // วาดเส้นและหัวลูกศร
    ctx.save();
    ctx.strokeStyle = '#7a0303e6';
    ctx.fillStyle = '#7a0303e6';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(startScreenX, startScreenY);
    ctx.lineTo(endScreenX, endScreenY);
    ctx.stroke();
    
    const angle = Math.atan2(endScreenY - startScreenY, endScreenX - startScreenX);
    const headlen = 10;
    ctx.beginPath();
    ctx.moveTo(endScreenX, endScreenY);
    ctx.lineTo(endScreenX - headlen * Math.cos(angle - Math.PI / 6), endScreenY - headlen * Math.sin(angle - Math.PI / 6));
    ctx.lineTo(endScreenX - headlen * Math.cos(angle + Math.PI / 6), endScreenY - headlen * Math.sin(angle + Math.PI / 6));
    ctx.closePath();
    ctx.stroke();
    ctx.fill();
    ctx.restore();
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
      // คำนวณหาจุดกึ่งกลางและสร้าง "หุ่นยนต์ตัวอย่าง"
            const meta = current_map_select.meta;
            const centerX = meta.origin[0] + (mapImage.width / 2) * meta.resolution;
            const centerY = meta.origin[1] + (mapImage.height / 2) * meta.resolution;
            tempInitialPose = {
                position: { x: centerX, y: centerY, z: 0 },
                orientation: { x: 0, y: 0, z: 0, w: 1 } // หันหน้าไปทางแกน X (ทิศตะวันออก)
            };
      resetStaticMapView(); 
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

function yawToQuaternion(yaw) {
  const halfYaw = yaw / 2.0;
  return {
    x: 0,
    y: 0,
    z: Math.sin(halfYaw),
    w: Math.cos(halfYaw),
  };
}

function getWorldCoordsFromEvent(e) {
  if (!activeMap.meta || !mapImage) return null;
  const rect = canvas.getBoundingClientRect();
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

function drawLaserScan() {
  if (!latestScan || !robotPose.position || !activeMap.meta || !mapImage) return;

  const { resolution, origin } = activeMap.meta;
  const mapImgHeight = mapImage.height;
  const robotYaw = getYawFromQuaternion(robotPose.orientation);

  ctx.fillStyle = 'rgba(255, 0, 255, 0.7)'; // สีชมพูโปร่งแสง

  latestScan.ranges.forEach((range, index) => {
    if (range < 0.1 || range > 10.0) return;

    const angle = latestScan.angle_min + index * latestScan.angle_increment;
    const totalAngle = robotYaw + angle;
    
    const worldX = robotPose.position.x + range * Math.cos(totalAngle);
    const worldY = robotPose.position.y + range * Math.sin(totalAngle);

    // แปลง World Coordinates เป็น Map Pixel Coordinates
    const px = (worldX - origin[0]) / resolution;
    const py = mapImgHeight - ((worldY - origin[1]) / resolution);
    
    ctx.beginPath();
    ctx.arc(px, py, 1.5 / mapView.viewState.scale, 0, 2 * Math.PI);
    ctx.fill();
  });
}

function drawRobot() {
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
  
  ctx.restore();
}

function getYawFromQuaternion(q) {
    if (!q) return 0;
    const { x, y, z, w } = q;
    return Math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

function drawTempInitialPose() {
    if (!tempInitialPose || !activeMap.meta || !mapImage) return;

    const { resolution, origin } = activeMap.meta;
    const mapImgHeight = mapImage.height;

    // แปลง World Coordinate เป็น Map Pixel Coordinate
    const px = (tempInitialPose.position.x - origin[0]) / resolution;
    const py = mapImgHeight - ((tempInitialPose.position.y - origin[1]) / resolution);
    const yaw = getYawFromQuaternion(tempInitialPose.orientation);

    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-yaw);

    const scale = 1.0 / mapView.viewState.scale;
    ctx.beginPath();
    ctx.moveTo(10 * scale, 0);
    ctx.lineTo(-5 * scale, -5 * scale);
    ctx.lineTo(-5 * scale, 5 * scale);
    ctx.closePath();
    ctx.fillStyle = 'rgba(255, 165, 0, 0.7)'; // สีส้ม โปร่งแสง
    ctx.strokeStyle = 'white';
    ctx.lineWidth = 0.5 * scale;
    ctx.fill();
    ctx.stroke();

    ctx.restore();
}
