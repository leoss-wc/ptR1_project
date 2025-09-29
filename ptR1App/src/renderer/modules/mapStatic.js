// modules/mapStatic.js
import { patrolPath } from './patrolState.js';
import { activeMap } from './mapState.js';
import * as mapView from './mapView.js'; 

let canvas, ctx, mapImage;
let isDrawing = false;
let isHoveringFirstPoint = false;
let current_map_select = { name: null, base64: null ,meta:null};
let goalPoint = null;
let mode = 'none';

let isSettingPose = false; 
let poseStartPosition = null;
let currentMousePos = { x: 0, y: 0 }; //เก็บตำแหน่งเมาส์ล่าสุดบน Canvas

let mapHitCanvas, mapHitCtx; //ตัวแปรสำหรับ Canvas ที่ใช้ตรวจสอบการคลิกแบบ Pixel-perfect

let dimmerMaskImage = null;//ตัวแปรสำหรับเก็บภาพมาสก์ Dimmer ที่สร้างขึ้น

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

function isClickInsideBounds(clickX, clickY) {
  // 1. ตรวจสอบว่า Canvas สำหรับตรวจสอบพร้อมใช้งานหรือไม่
  if (!mapHitCtx) return false;

  // 2. แปลงพิกัดบนหน้าจอ (Screen) เป็นพิกัดบน "รูปภาพแผนที่" (Image Pixel)
  // ของเดิม: const px = Math.floor((clickX - offsetX) / zoom);
  const px = Math.floor((clickX - mapView.viewState.offsetX) / mapView.viewState.scale); 
  // ของเดิม: const py = Math.floor((clickY - offsetY) / zoom);
  const py = Math.floor((clickY - mapView.viewState.offsetY) / mapView.viewState.scale);

  // 3. ตรวจสอบว่าพิกัดที่คำนวณได้อยู่นอกขอบเขตของรูปภาพหรือไม่
  if (px < 0 || px >= mapHitCanvas.width || py < 0 || py >= mapHitCanvas.height) {
    return false;
  }

  // 4. ดึงข้อมูลสี (RGBA) จากพิกัดนั้นบน Canvas อ้างอิง
  const pixelData = mapHitCtx.getImageData(px, py, 1, 1).data;
  const colorValue = pixelData[0]; // สำหรับภาพ Grayscale ค่า R, G, B จะเท่ากัน

  // 5. กำหนดค่า Threshold สำหรับพื้นที่ว่าง (ปกติสีขาวจะมีค่าใกล้ 255)
  const freeSpaceThreshold = 250; 
  
  // 6. คืนค่า true ถ้าสีของพิกเซลนั้นสว่างกว่าค่าที่กำหนด (เป็นพื้นที่ว่าง)
  return colorValue > freeSpaceThreshold;
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
  
  // 3. สร้างรูปทรงของรู (เป็นพิกัดพิกเซลของแผนที่)
  ctx.moveTo(0, 0);
  ctx.lineTo(mapImage.width, 0);
  ctx.lineTo(mapImage.width, mapImage.height);
  ctx.lineTo(0, mapImage.height);
  ctx.closePath();
  
  // 4. คืนค่าการแปลงค่าทั้งหมดกลับสู่ปกติ
  ctx.restore(); 

  // 5. เติมสีโดยใช้ 'evenodd' rule ซึ่งจะเว้นพื้นที่รูที่ถูกแปลงค่าไว้
  ctx.fill("evenodd");
}
/*
function setupCanvasEvents() {
  canvas.addEventListener('mousedown', (e) => {
    const rect = canvas.getBoundingClientRect();
    const clickX = e.clientX - rect.left;
    const clickY = e.clientY - rect.top;
    if (!isClickInsideBounds(clickX, clickY)) return;
    if (mode === 'draw') {
      if (isHoveringFirstPoint && patrolPath.length > 1) {
        patrolPath.push({ ...patrolPath[0] });
        renderCanvas();
        cancelMode();
      } else {
        isDrawing = true;
        addPathPoint(e); // จะเรียกใช้ addPathPoint ที่ถูกต้อง
        renderCanvas();
      }
    } else if (mode === 'goal') {
      if (!isClickInsideBounds(clickX, clickY)) return;
      setGoalPointOnClick(e);
    }else if (mode === 'pose') {
      isSettingPose = true;
      poseStartPosition = getWorldCoordsFromEvent(e);
      renderCanvas(); // วาดจุดเริ่มต้น
    }else {
      isDrawing = false;
      isDragging = true;
      dragStartX = e.clientX;
      dragStartY = e.clientY;
    }
  });
  canvas.addEventListener('mouseup', (e) => {
    if (isSettingPose) {
      const endPoint = getWorldCoordsFromEvent(e);
      
      // คำนวณมุมจากจุดเริ่มต้นไปยังจุดที่ปล่อยเมาส์
      const dx = endPoint.x - poseStartPosition.x;
      const dy = endPoint.y - poseStartPosition.y;
      const yaw = Math.atan2(dy, dx);
      
      const quaternion = yawToQuaternion(yaw);
      
      const poseData = {
        position: poseStartPosition,
        orientation: quaternion,
      };

      // ส่งข้อมูลไปให้ Backend
      window.electronAPI.setInitialPose(poseData);
      
      // ออกจากโหมด
      isSettingPose = false;
      poseStartPosition = null;
      cancelMode();
    }
    isDrawing = false;
    isDragging = false;
  });
  canvas.addEventListener('mouseleave', () => {
    isDrawing = false;
    isDragging = false;
    if (isHoveringFirstPoint) {
      isHoveringFirstPoint = false;
      renderCanvas();
    }
  });
  canvas.addEventListener('mousemove', (e) => {

    const rect = canvas.getBoundingClientRect();
    currentMousePos.x = e.clientX - rect.left;
    currentMousePos.y = e.clientY - rect.top;

    if (isSettingPose) {
      renderCanvas(); // วาดใหม่เพื่อให้เห็นลูกศรตามเมาส์
    }
    if (mode === 'draw') {
        if (isDrawing) {
            addPathPoint(e);
            renderCanvas();
        } else if (patrolPath.length > 0 && activeMap.meta) {
            // Hover logic (เหมือนเดิม)
            const snapRadius = 10 / zoom;
            const firstPoint = patrolPath[0];
            const { resolution, origin } = activeMap.meta;
            const firstPointMapX = (firstPoint.x - origin[0]) / resolution;
            const firstPointMapY = mapImage.height - (firstPoint.y - origin[1]) / resolution;
            const rect = canvas.getBoundingClientRect();
            const mouseScreenX = e.clientX - rect.left;
            const mouseScreenY = e.clientY - rect.top;
            const mouseMapX = (mouseScreenX - offsetX) / zoom;
            const mouseMapY = (mouseScreenY - offsetY) / zoom;
            const distance = Math.sqrt(Math.pow(mouseMapX - firstPointMapX, 2) + Math.pow(mouseMapY - firstPointMapY, 2));
            const previouslyHovering = isHoveringFirstPoint;
            isHoveringFirstPoint = distance < snapRadius;
            if (previouslyHovering !== isHoveringFirstPoint) {
                canvas.style.cursor = isHoveringFirstPoint ? 'pointer' : 'crosshair';
                renderCanvas();
            }
        }
    }
    if (isDragging) {
      const dx = e.clientX - dragStartX;
      const dy = e.clientY - dragStartY;
      dragStartX = e.clientX;
      dragStartY = e.clientY;
      offsetX += dx;
      offsetY += dy;
      renderCanvas();
    }

  });
  canvas.addEventListener('wheel', (e) => {
    if (!mapImage) return;
    e.preventDefault();
    const rect = canvas.getBoundingClientRect();
    const mouseX = e.clientX - rect.left;
    const mouseY = e.clientY - rect.top;
    const mapXBeforeZoom = (mouseX - offsetX) / zoom;
    const mapYBeforeZoom = (mouseY - offsetY) / zoom;
    const zoomFactor = e.deltaY < 0 ? 1.2 : 1 / 1.2;
    zoom *= zoomFactor;
    offsetX = mouseX - mapXBeforeZoom * zoom;
    offsetY = mouseY - mapYBeforeZoom * zoom;
    renderCanvas();
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
  */

function setupCanvasEvents() {
  canvas.addEventListener('mousedown', (e) => {
    if (mode === 'draw' || mode === 'goal' || mode === 'pose') {
      // --- Logic สำหรับโหมดพิเศษ (เหมือนเดิม) ---
      const rect = canvas.getBoundingClientRect();
      const clickX = e.clientX - rect.left;
      const clickY = e.clientY - rect.top;
      if (!isClickInsideBounds(clickX, clickY)) return;

      if (mode === 'draw') {
        if (isHoveringFirstPoint && patrolPath.length > 1) { 
        patrolPath.push({ ...patrolPath[0] });
        renderCanvas();
        cancelMode(); 
      } else { 
        isDrawing = true; 
        addPathPoint(e); 
        renderCanvas(); 
      }

      } else if (mode === 'goal') {
        setGoalPointOnClick(e);
      } else if (mode === 'pose') {
        isSettingPose = true;
        poseStartPosition = getWorldCoordsFromEvent(e);
        renderCanvas();
      }
    } else {
      // --- ✨ เมื่อเป็นโหมดปกติ ให้เรียกใช้ Pan Logic จาก mapView ---
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
      
      isSettingPose = false;
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

    // แยก Logic การทำงานตาม mode อย่างชัดเจน
    if (mode === 'draw') {
      if (isDrawing) {
        // ถ้ากำลังวาด: เพิ่มจุดและวาดใหม่
        addPathPoint(e);
        renderCanvas();
      } else if (patrolPath.length > 0 && activeMap.meta) {
        const snapRadius = 10 / mapView.viewState.scale;
        const firstPoint = patrolPath[0];
        const { resolution, origin } = activeMap.meta;
        
        // --- 🔧 แก้ไขสูตรคำนวณ 2 บรรทัดนี้ ---
        const firstPointPy = (firstPoint.x - origin[0]) / resolution; // world x -> pixel y
        const firstPointPx = mapImage.width - ((firstPoint.y - origin[1]) / resolution); // world y -> pixel x
        
        const mouseMapX = (currentMousePos.x - mapView.viewState.offsetX) / mapView.viewState.scale;
        const mouseMapY = (currentMousePos.y - mapView.viewState.offsetY) / mapView.viewState.scale;
        
        // --- 🔧 อัปเดตตัวแปรที่ใช้คำนวณ distance ---
        const distance = Math.sqrt(Math.pow(mouseMapX - firstPointPx, 2) + Math.pow(mouseMapY - firstPointPy, 2));
        const previouslyHovering = isHoveringFirstPoint;
        isHoveringFirstPoint = distance < snapRadius;

        if (previouslyHovering !== isHoveringFirstPoint) {
            canvas.style.cursor = isHoveringFirstPoint ? 'pointer' : 'crosshair';
            renderCanvas();
        }
      }
    } else if (mode === 'pose') {
      // ถ้าอยู่ในโหมด pose: วาดลูกศรตามเมาส์
      if (isSettingPose) {
        renderCanvas();
      }
    } else {
      // ถ้าเป็นโหมดปกติ (none): ให้ Pan แผนที่
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
  poseStartPosition = null;
  canvas.style.cursor = 'grab';
  document.getElementById('set-goal-btn').classList.remove('active');
  const drawModeBtn = document.getElementById('toggle-draw-mode');
  drawModeBtn.textContent = 'Draw :OFF';
  drawModeBtn.classList.remove('active');
  renderCanvas();
}

function addPathPoint(e) {
  if (!activeMap.meta || !mapImage || mapImage.height === 0) return;
  const rect = canvas.getBoundingClientRect();
  const { resolution, origin } = activeMap.meta;

  const clickX = e.clientX - rect.left;
  const clickY = e.clientY - rect.top;

  const px = (clickX - mapView.viewState.offsetX) / mapView.viewState.scale;
  const py = (clickY - mapView.viewState.offsetY) / mapView.viewState.scale;

  const worldPoint = {
        x: origin[0] + (px * resolution),
        y: origin[1] + ((mapImage.height - py) * resolution)
    };

  patrolPath.push(worldPoint);
}

export function renderCanvas() {
  if (!canvas || !mapImage) return;

  // 1. เตรียม Canvas และล้างหน้าจอ
  canvas.width = canvas.clientWidth;
  canvas.height = canvas.clientHeight;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.imageSmoothingEnabled = false;

  // 2. บันทึกสถานะเริ่มต้นของ Canvas ก่อนทำการแปลงใดๆ
  ctx.save();

  // 3. ใช้ mapView จัดการ Pan และ Zoom (จะมีการ translate และ scale context)
  ctx.translate(mapView.viewState.offsetX, mapView.viewState.offsetY);
  ctx.scale(mapView.viewState.scale, mapView.viewState.scale);

  // 4. หมุนมุมมองทั้งหมด -90 องศา เพื่อให้แผนที่ตั้งตรง
  ctx.rotate(-Math.PI / 2);
  ctx.translate(-mapImage.height, 0); // ย้ายภาพกลับเข้ามาในกรอบหลังหมุน

  // --- ณ จุดนี้ ระบบพิกัดพร้อมแล้ว ทุกอย่างที่วาดหลังจากนี้จะถูกแปลงโดยอัตโนมัติ ---

  // 5. วาดแผนที่ (ที่ยังเอียงอยู่) ลงบนมุมมองที่ถูกหมุน -> ผลลัพธ์คือภาพที่ตั้งตรง
  ctx.drawImage(mapImage, 0, 0, mapImage.width, mapImage.height);

   if (dimmerMaskImage && (mode === 'draw' || mode === 'goal')) {
    ctx.drawImage(dimmerMaskImage, 0, 0, mapImage.width, mapImage.height);
  }
  // 6. วาดทุกอย่างที่เหลือ (Path, Goal, etc.) โดยใช้ "พิกัดพิกเซลของแผนที่ดั้งเดิม"
  if (activeMap.meta) {
    const { resolution, origin } = activeMap.meta;
    const mapImgHeight = mapImage.height;

    // --- วาดเส้น Path ---
    if (patrolPath.length > 1) {
      ctx.strokeStyle = 'orange';
      ctx.lineWidth = 2 / mapView.viewState.scale; // ปรับความหนาเส้นตามการซูม
      ctx.beginPath();
      patrolPath.forEach((point, i) => {
        // แปลง World Coords -> Map Pixel Coords (สูตรดั้งเดิม)
        const px = (point.x - origin[0]) / resolution;
        const py = mapImgHeight - ((point.y - origin[1]) / resolution);
        // วาดลงไปตรงๆ ที่ px, py (Context ที่ถูกแปลงแล้วจะจัดการตำแหน่งบนจอให้เอง)
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

    // --- วาด Goal Point ---
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
  }

  // 7. คืนค่า Canvas กลับสู่สถานะเริ่มต้น (ไม่มี Pan, Zoom, หรือ Rotation)
  ctx.restore();
  if (mode === 'draw' || mode === 'goal') {
    drawBoundaryMask();
  }

  // --- ณ จุดนี้ เรากลับมาวาดใน "พิกัดหน้าจอ" (Screen Space) ตามปกติ ---
  
  // 8. วาดลูกศร Initial Pose (ซึ่งต้องสัมพันธ์กับตำแหน่งเมาส์บนหน้าจอ)
  if (isSettingPose && poseStartPosition) {
    // แปลงจุดเริ่มต้น (World Coords) ให้ออกมาเป็นพิกัดบนหน้าจอ (Screen Coords)
    const { resolution, origin } = activeMap.meta;
    const startPx = (poseStartPosition.x - origin[0]) / resolution;
    const startPy = mapImage.height - ((poseStartPosition.y - origin[1]) / resolution);
    
    // คำนวณตำแหน่งบนจอโดยจำลองการแปลงทั้งหมด
    const rotatedX = startPy;
    const rotatedY = -startPx + mapImage.height;
    
    const startScreenX = rotatedX * mapView.viewState.scale + mapView.viewState.offsetX;
    const startScreenY = rotatedY * mapView.viewState.scale + mapView.viewState.offsetY;

    // จุดสิ้นสุดคือตำแหน่งเมาส์ปัจจุบัน
    const endScreenX = currentMousePos.x;
    const endScreenY = currentMousePos.y;

    // วาดลูกศร
    ctx.save();
    ctx.strokeStyle = 'rgba(0, 255, 0, 0.9)';
    ctx.fillStyle = 'rgba(0, 255, 0, 0.7)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(startScreenX, startScreenY);
    ctx.lineTo(endScreenX, endScreenY);
    ctx.stroke();
    // ... (โค้ดวาดหัวลูกศร) ...
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

  // 5. คืนค่าเป็นรูปภาพและ metadata ที่ผ่านการหมุนแล้ว
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
      // ไม่ต้องทำ processing ใดๆ แค่แสดงผล
      resetStaticMapView(); 
      resetStaticMapView(); 
    };
    mapImage.src = base64;
  });
  document.getElementById('map-gallery').appendChild(img);
}

function setGoalPointOnClick(e) {
  if (!activeMap.meta || !mapImage || mapImage.height === 0) return;
  const rect = canvas.getBoundingClientRect();
  const clickX = e.clientX - rect.left;
  const clickY = e.clientY - rect.top;

  const px = (clickX - mapView.viewState.offsetX) / mapView.viewState.scale;
  const py = (clickY - mapView.viewState.offsetY) / mapView.viewState.scale;
  
  // --- 🔧 ใช้สูตรคำนวณใหม่ทั้งหมด ---
  goalPoint = {
    x: activeMap.meta.origin[0] + (px * activeMap.meta.resolution),
    y: activeMap.meta.origin[1] + ((mapImage.height - py) * activeMap.meta.resolution)
  };
  
  window.electronAPI.sendSingleGoal(goalPoint);
  cancelMode();
  renderCanvas();
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

// ✨ เพิ่ม: ฟังก์ชันสำหรับแปลง Buffer เป็น Base64
function bufferToBase64(buffer) {
    let binary = '';
    const bytes = new Uint8Array(buffer);
    const len = bytes.byteLength;
    for (let i = 0; i < len; i++) {
        binary += String.fromCharCode(bytes[i]);
    }
    return window.btoa(binary);
}

// ✨ เพิ่ม: ฟังก์ชันสำหรับแปลง Base64 กลับเป็น Uint8ClampedArray
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
  if (!activeMap.meta || !mapImage || mapImage.height === 0) return null;
  const rect = canvas.getBoundingClientRect();
  const clickX = e.clientX - rect.left;
  const clickY = e.clientY - rect.top;

  const px = (clickX - mapView.viewState.offsetX) / mapView.viewState.scale;
  const py = (clickY - mapView.viewState.offsetY) / mapView.viewState.scale;
  return {
    x: activeMap.meta.origin[0] + (px * activeMap.meta.resolution),
    y: activeMap.meta.origin[1] + ((mapImage.height - py) * activeMap.meta.resolution)
  };
}
