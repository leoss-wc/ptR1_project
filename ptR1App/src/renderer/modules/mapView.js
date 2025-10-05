// src/renderer/modules/mapView.js (เวอร์ชันใหม่)

export const viewState = {
  scale: 1.0,
  offsetX: 0,
  offsetY: 0,
};

let isDragging = false;
let lastX = 0;
let lastY = 0;
let redrawCallback = null; // ฟังก์ชันสำหรับวาดใหม่
let redrawLiveCallback = null;
let redrawStaticCallback = null;

// ฟังก์ชันสำหรับตั้งค่าการซูมและ callback
export function initMapViewController(container, staticRedrawAllCb, liveRedrawAllCb) {
  redrawStaticCallback = staticRedrawAllCb;
  redrawLiveCallback = liveRedrawAllCb;

  container.addEventListener('wheel', (e) => {
    e.preventDefault();
    const scaleAmount = 1.1;
    const rect = container.getBoundingClientRect();
    const mouseX = e.clientX - rect.left;
    const mouseY = e.clientY - rect.top;
    const mapX = (mouseX - viewState.offsetX) / viewState.scale;
    const mapY = (mouseY - viewState.offsetY) / viewState.scale;
    
    if (e.deltaY < 0) { viewState.scale *= scaleAmount; }
    else { viewState.scale /= scaleAmount; }

    viewState.offsetX = mouseX - mapX * viewState.scale;
    viewState.offsetY = mouseY - mapY * viewState.scale;

    // เรียก callback ที่ถูกต้อง
    if (redrawStaticCallback) redrawStaticCallback();
    if (redrawLiveCallback) redrawLiveCallback();
  });
}

// --- ฟังก์ชันที่ `mapStatic.js` จะเรียกใช้ ---

export function handleMouseDown(e) {
  isDragging = true;
  lastX = e.clientX;
  lastY = e.clientY;
  e.target.style.cursor = 'grabbing';
}

export function handleMouseMove(e) {
  if (!isDragging) return;
  const dx = e.clientX - lastX;
  const dy = e.clientY - lastY;
  viewState.offsetX += dx;
  viewState.offsetY += dy;
  lastX = e.clientX;
  lastY = e.clientY;
  
  // เรียก callback ที่ถูกตั้งค่าไว้ตอน init
  if (redrawStaticCallback) redrawStaticCallback();
  if (redrawLiveCallback) redrawLiveCallback();
}


export function handleMouseUp(e) {
  isDragging = false;
  e.target.style.cursor = 'grab';
}

// --- ฟังก์ชันจัดการ Transform

export function resetView() {
  viewState.scale = 1.0;
  viewState.offsetX = 0;
  viewState.offsetY = 0;
}

export function applyTransform(ctx) {
  ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
  ctx.save();
  ctx.translate(viewState.offsetX, viewState.offsetY);
  ctx.scale(viewState.scale, viewState.scale);
}

export function restoreTransform(ctx) {
  ctx.restore();
}