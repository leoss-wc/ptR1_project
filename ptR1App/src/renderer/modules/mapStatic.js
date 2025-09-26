// modules/mapStatic.js
import { patrolPath } from './patrolState.js';
import { activeMap } from './mapState.js';

let canvas, ctx, mapImage;
let zoom = 1.0;
let offsetX = 0;
let offsetY = 0;
let isDragging = false;
let isDrawing = false;
let dragStartX = 0;
let dragStartY = 0;
let isHoveringFirstPoint = false;
let current_map_select = { name: null, base64: null ,meta:null};
let goalPoint = null;
let mode = 'none';

let isSettingPose = false; 
let poseStartPosition = null;
let currentMousePos = { x: 0, y: 0 }; //เก็บตำแหน่งเมาส์ล่าสุดบน Canvas

let mapHitCanvas, mapHitCtx; //ตัวแปรสำหรับ Canvas ที่ใช้ตรวจสอบการคลิกแบบ Pixel-perfect

let dimmerMaskImage = null;//ตัวแปรสำหรับเก็บภาพมาสก์ Dimmer ที่สร้างขึ้น
let processedSelection = null; // ตัวแปรสำหรับเก็บผลลัพธ์ที่ประมวลผลเสร็จแล้วชั่วคราว

export function initStaticMap() {
  canvas = document.getElementById('staticMapCanvas');
  ctx = canvas.getContext('2d');
  bindUI();
  setupCanvasEvents();
  loadLocalMapsToGallery();
}

// ✨ เพิ่ม: ฟังก์ชันสำหรับ Reset View โดยใช้หลัก "Fit and Center"
function resetStaticMapView() {
  if (!canvas || !mapImage) return;
  console.log("🗺️ StaticMap: View reset to fit and center.");
  const zoomX = canvas.width / mapImage.width;
  const zoomY = canvas.height / mapImage.height;
  zoom = Math.min(zoomX, zoomY) * 0.95; // ซูมออกเล็กน้อยให้มีขอบ
  offsetX = (canvas.width - mapImage.width * zoom) / 2;
  offsetY = (canvas.height - mapImage.height * zoom) / 2;
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
      zoom *= 1.2;
      renderCanvas();
    }
  });
  document.getElementById('zoom-out').addEventListener('click', () => {
    if (mapImage) {
      zoom /= 1.2;
      renderCanvas();
    }
  });
  // 🔧 แก้ไข: ให้ปุ่ม Reset เรียกใช้ฟังก์ชันใหม่
  document.getElementById('reset-view').addEventListener('click', resetStaticMapView);

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
  const px = Math.floor((clickX - offsetX) / zoom);
  const py = Math.floor((clickY - offsetY) / zoom);

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

  ctx.save(); // บันทึกสถานะ Canvas ปัจจุบัน

  // --- ส่วนที่ 1: ทำให้พื้นที่ "นอก" แผนที่มืดลง ---
  ctx.fillStyle = "rgba(0, 0, 0, 0.6)";
  ctx.beginPath();
  // สร้าง Path เต็มพื้นที่ Canvas
  ctx.rect(0, 0, canvas.width, canvas.height);
  // สร้าง Path "เจาะรู" ตรงกลางเป็นรูปสี่เหลี่ยมของแผนที่ (ทวนเข็มนาฬิกา)
  ctx.moveTo(offsetX, offsetY);
  ctx.lineTo(offsetX, offsetY + mapImage.height * zoom);
  ctx.lineTo(offsetX + mapImage.width * zoom, offsetY + mapImage.height * zoom);
  ctx.lineTo(offsetX + mapImage.width * zoom, offsetY);
  ctx.closePath();
  // ใช้ 'evenodd' rule เพื่อเติมสีเฉพาะพื้นที่ที่ไม่มีรูเจาะ (คือพื้นที่ด้านนอก)
  ctx.fill("evenodd");

  // --- ส่วนที่ 2: วาด Dimmer "ใน" แผนที่ (Pixel-perfect) ---
  if (dimmerMaskImage) {
    ctx.drawImage(dimmerMaskImage, offsetX, offsetY, mapImage.width * zoom, mapImage.height * zoom);
  }

  ctx.restore(); // คืนค่าสถานะ Canvas
}

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
  const imgH = mapImage.height;
  const clickX = e.clientX - rect.left;
  const clickY = e.clientY - rect.top;
  const px = (clickX - offsetX) / zoom;
  const py = (clickY - offsetY) / zoom;
  const worldPoint = {
    x: origin[0] + (px * resolution),
    y: origin[1] + ((imgH - py) * resolution)
  };
  patrolPath.push(worldPoint);
}

function renderCanvas() {
  if (!canvas) return;
  canvas.width = canvas.clientWidth;
  canvas.height = canvas.clientHeight;
  if (!mapImage) return;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.drawImage(mapImage, offsetX, offsetY, mapImage.width * zoom, mapImage.height * zoom);

  
  // --- ลำดับที่ 2: วาด Dimmer Effect (ถ้าอยู่ในโหมดที่กำหนด) ---
  if (mode === 'draw' || mode === 'goal') {
    drawBoundaryMask();
  }

  // --- ลำดับที่ 3: วาดเส้น Path, จุด, และ Goal ทับลงไปบนสุด ---
  if (current_map_select.name) {
    ctx.font = '16px sans-serif';
    ctx.fillStyle = 'rgba(255, 255, 255, 0.8)';
    ctx.fillText(`Map: ${current_map_select.name}`, 10, 20);
  }

  if (patrolPath.length > 0 && activeMap.meta) {
    const { resolution, origin } = activeMap.meta;
    if (patrolPath.length > 1) {
      ctx.strokeStyle = 'orange';
      ctx.lineWidth = 2;
      ctx.beginPath();
      patrolPath.forEach((point, i) => {
        const px = (point.x - origin[0]) / resolution;
        const py = mapImage.height - (point.y - origin[1]) / resolution;
        const screenX = px * zoom + offsetX;
        const screenY = py * zoom + offsetY;
        i === 0 ? ctx.moveTo(screenX, screenY) : ctx.lineTo(screenX, screenY);
      });
      ctx.stroke();
    }

    const baseRadius = 6;         // ขนาดพื้นฐานของจุด
    const hoverRadius = 8;        // ขนาดพื้นฐานของจุดเมื่อถูกชี้
    const minRadius = 2;          // ขนาดเล็กที่สุดที่อนุญาต (pixel)
    const maxRadius = 8;         // ขนาดใหญ่ที่สุดที่อนุญาต (pixel)


    patrolPath.forEach((point, i) => {
      const px = (point.x - origin[0]) / resolution;
      const py = mapImage.height - (point.y - origin[1]) / resolution;
      const screenX = px * zoom + offsetX;
      const screenY = py * zoom + offsetY;
      ctx.beginPath();

      if (i === 0 && isHoveringFirstPoint) {
        // --- คำนวณขนาดของจุดที่ถูกชี้ (Hover) ---
        const calculatedRadius = hoverRadius / zoom;
        const radius = Math.max(minRadius, Math.min(calculatedRadius, maxRadius));
        
        ctx.fillStyle = '#00FF00';
        ctx.arc(screenX, screenY, radius, 0, Math.PI * 2);

      } else {
        // --- คำนวณขนาดของจุดปกติ ---
        const calculatedRadius = baseRadius / zoom;
        const radius = Math.max(minRadius, Math.min(calculatedRadius, maxRadius));

        ctx.fillStyle = 'cyan';
        ctx.arc(screenX, screenY, radius, 0, Math.PI * 2);
      }


      ctx.fill();
    });
  }
  if (goalPoint) {
    const px = (goalPoint.x - activeMap.meta.origin[0]) / activeMap.meta.resolution;
    const py = mapImage.height - (goalPoint.y - activeMap.meta.origin[1]) / activeMap.meta.resolution;
    const x = px * zoom + offsetX;
    const y = py * zoom + offsetY;
    ctx.beginPath();
    ctx.arc(x, y, 6 / zoom, 0, 2 * Math.PI);
    ctx.fillStyle = 'red';
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = 'white';
    ctx.stroke();
  }

  if (isSettingPose && poseStartPosition) {
    // 1. แปลง World Coords ของจุดเริ่มต้นกลับเป็น Screen Coords
    const startPx = (poseStartPosition.x - activeMap.meta.origin[0]) / activeMap.meta.resolution;
    const startPy = mapImage.height - (poseStartPosition.y - activeMap.meta.origin[1]) / activeMap.meta.resolution;
    const startScreenX = startPx * zoom + offsetX;
    const startScreenY = startPy * zoom + offsetY;

    // 2. จุดสิ้นสุดคือตำแหน่งเมาส์ปัจจุบัน (ซึ่งเป็น Screen Coords อยู่แล้ว)
    const endScreenX = currentMousePos.x;
    const endScreenY = currentMousePos.y;

    // 3. วาดเส้นและหัวลูกศร
    ctx.save();
    ctx.strokeStyle = 'rgba(0, 255, 0, 0.9)';
    ctx.fillStyle = 'rgba(0, 255, 0, 0.7)';
    ctx.lineWidth = Math.max(1, 3 / zoom); // ทำให้เส้นไม่หนาหรือบางเกินไป

    // วาดเส้นตรง
    ctx.beginPath();
    ctx.moveTo(startScreenX, startScreenY);
    ctx.lineTo(endScreenX, endScreenY);
    ctx.stroke();

    // วาดหัวลูกศร
    const angle = Math.atan2(endScreenY - startScreenY, endScreenX - startScreenX);
    const headlen = Math.max(5, 15 / zoom); // ขนาดหัวลูกศร
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

  // 2. คำนวณขนาดและสร้าง Canvas ใหม่สำหรับรูปที่ Crop แล้ว
  const cropWidth = maxX - minX + 1;
  const cropHeight = maxY - minY + 1;
  const cropCanvas = document.createElement('canvas');
  cropCanvas.width = cropWidth;
  cropCanvas.height = cropHeight;
  const cropCtx = cropCanvas.getContext('2d');

  // 3. วาดเฉพาะส่วนของแผนที่ลงบน Canvas ใหม่
  cropCtx.drawImage(sourceImage, minX, minY, cropWidth, cropHeight, 0, 0, cropWidth, cropHeight);
  
  // 4. สร้าง Image object ใหม่จาก Canvas ที่ Crop แล้ว
  const croppedImage = new Image();
  croppedImage.src = cropCanvas.toDataURL();
  await new Promise(resolve => croppedImage.onload = resolve); // รอให้รูปใหม่โหลดเสร็จ

  // 5. คำนวณ Origin ใหม่ให้สอดคล้องกับรูปที่ถูกตัด
  const newMeta = JSON.parse(JSON.stringify(meta)); // Deep copy
  newMeta.origin[0] = meta.origin[0] + minX * meta.resolution;
  newMeta.origin[1] = meta.origin[1] + (height - maxY - 1) * meta.resolution;
  
  console.log(`✅ Cropping complete. New size: ${cropWidth}x${cropHeight}. New origin:`, newMeta.origin);
  return { croppedImage, newMeta };
}

function addMapToGallery(name, base64) {
  const img = document.createElement('img');
  img.src = base64;
  img.alt = name;
  img.title = name;
  img.className = 'map-thumb';
  img.style.cursor = 'pointer';
  img.addEventListener('click', async () => {
    // 🔧 แก้ไข: ส่วนนี้จะทำแค่ "Preview"
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
  const px = (clickX - offsetX) / zoom;
  const py = (clickY - offsetY) / zoom;
  goalPoint = {
    x: activeMap.meta.origin[0] + (px * activeMap.meta.resolution),
    y: activeMap.meta.origin[1] + ((mapImage.height - py) * activeMap.meta.resolution)
  };
  window.electronAPI.sendSingleGoal(goalPoint);
  cancelMode();
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

//ฟังก์ชันย่อยสำหรับแปลง event เป็น world coords (ลดโค้ดซ้ำซ้อน)
function getWorldCoordsFromEvent(e) {
  if (!activeMap.meta || !mapImage || mapImage.height === 0) return null;
  const rect = canvas.getBoundingClientRect();
  const clickX = e.clientX - rect.left;
  const clickY = e.clientY - rect.top;
  const px = (clickX - offsetX) / zoom;
  const py = (clickY - offsetY) / zoom;
  return {
    x: activeMap.meta.origin[0] + (px * activeMap.meta.resolution),
    y: activeMap.meta.origin[1] + ((mapImage.height - py) * activeMap.meta.resolution)
  };
}
