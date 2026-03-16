// ---- State ----------------------------------------------------------------
let _isBursting  = false;
let _captureCount = 0;

// ---- Init -----------------------------------------------------------------
export function initCaptureControl() {
  const ui = _getElements();
  if (!ui.panel) {
    console.warn('[captureControl] #capture-panel not found in DOM');
    return;
  }

  _loadStats();
  _bindEvents(ui);
  _listenCaptureResult(ui);

  console.log('[captureControl] Initialized');
}

// ---- Event Binding --------------------------------------------------------
function _bindEvents(ui) {

  // ปุ่ม Single Shot
  ui.btnSingle.addEventListener('click', async () => {
    const label = ui.labelInput.value.trim() || 'object';
    const res = await window.electronAPI.captureSnapshot(label);
    if (!res.success) _showStatus(ui, `Error: ${res.message}`, 'error');
  });

  // ปุ่ม Start Burst
  ui.btnBurst.addEventListener('click', async () => {
    const label    = ui.labelInput.value.trim() || 'object';
    const interval = parseFloat(ui.intervalInput.value) || 2.0;

    const res = await window.electronAPI.captureStartBurst({ label, interval });
    if (res.success) {
      _isBursting = true;
      _updateBurstUI(ui, true);
      _showStatus(ui, `Burst: capturing "${label}" every ${interval}s`, 'active');
    } else {
      _showStatus(ui, `Error: ${res.message}`, 'error');
    }
  });

  // ปุ่ม Stop Burst
  ui.btnStop.addEventListener('click', async () => {
    await window.electronAPI.captureStopBurst();
    _isBursting = false;
    _updateBurstUI(ui, false);
    _showStatus(ui, `Stopped. Total: ${_captureCount} images`, 'idle');
  });

  // ปุ่ม Open Folder
  ui.btnFolder.addEventListener('click', async () => {
    const label = ui.labelInput.value.trim() || '';
    await window.electronAPI.captureOpenFolder(label);
  });

  // ป้องกัน keydown ไปโดนหุ่นยนต์
  [ui.labelInput, ui.intervalInput].forEach(el => {
    el.addEventListener('keydown', e => e.stopPropagation());
    el.addEventListener('keyup',   e => e.stopPropagation());
  });
}

// ---- Listen capture:result -----------------------------------------------
function _listenCaptureResult(ui) {
  window.electronAPI.onCaptureResult((data) => {
    _captureCount = data.count;
    ui.counter.textContent = `${data.count} images`;
    ui.lastLabel.textContent = data.label;

    // Flash preview thumbnail ถ้ามี
    if (data.thumb_b64) {
      ui.preview.src = `data:image/jpeg;base64,${data.thumb_b64}`;
      ui.preview.classList.remove('hidden');
    }

    // Animate counter
    ui.counter.classList.add('capture-flash');
    setTimeout(() => ui.counter.classList.remove('capture-flash'), 300);
  });
}

// ---- Load Stats -----------------------------------------------------------
async function _loadStats() {
  const res = await window.electronAPI.captureGetStats();
  if (res.success && res.labels.length > 0) {
    const total = res.labels.reduce((sum, l) => sum + l.count, 0);
    _captureCount = total;
    // อัปเดต counter ถ้า element พร้อม
    const counterEl = document.getElementById('capture-count');
    if (counterEl) counterEl.textContent = `${total} images`;
  }
}

// ---- UI Helpers -----------------------------------------------------------
function _updateBurstUI(ui, isBursting) {
  ui.btnBurst.disabled   = isBursting;
  ui.btnStop.disabled    = !isBursting;
  ui.btnSingle.disabled  = isBursting;
  ui.labelInput.disabled = isBursting;
  ui.intervalInput.disabled = isBursting;
  ui.indicator.className = isBursting ? 'capture-indicator active' : 'capture-indicator';
}

function _showStatus(ui, message, type = 'idle') {
  ui.statusText.textContent = message;
  ui.statusText.className   = `capture-status capture-status--${type}`;
}

function _getElements() {
  return {
    panel:         document.getElementById('capture-panel'),
    labelInput:    document.getElementById('capture-label'),
    intervalInput: document.getElementById('capture-interval'),
    btnSingle:     document.getElementById('capture-single'),
    btnBurst:      document.getElementById('capture-burst'),
    btnStop:       document.getElementById('capture-stop'),
    btnFolder:     document.getElementById('capture-open-folder'),
    counter:       document.getElementById('capture-count'),
    lastLabel:     document.getElementById('capture-last-label'),
    preview:       document.getElementById('capture-preview'),
    statusText:    document.getElementById('capture-status'),
    indicator:     document.getElementById('capture-indicator'),
  };
}