// detectionSettings.js — logic only
export function initDetectionSettings() {

  // Enable toggle label
  document.getElementById('det-enabled').addEventListener('change', (e) => {
    document.getElementById('det-enabled-label').textContent =
      e.target.checked ? 'Enabled' : 'Disabled';
  });

  // Mode → show/hide time section
  document.querySelectorAll('input[name="det-mode"]').forEach(radio => {
    radio.addEventListener('change', (e) => {
      document.getElementById('det-time-section').style.display =
        e.target.value === 'time' ? '' : 'none';
    });
  });

  // Time inputs → update preview
  ['det-time-start', 'det-time-end'].forEach(id => {
    document.getElementById(id).addEventListener('input', updateTimePreview);
  });

  // Update button
  document.getElementById('det-update-btn').addEventListener('click', handleUpdate);
}

function updateTimePreview() {
  const s = parseInt(document.getElementById('det-time-start').value, 10);
  const e = parseInt(document.getElementById('det-time-end').value, 10);
  const fmt = (h) => String(h).padStart(2, '0') + ':00';
  document.getElementById('det-time-preview').textContent = `${fmt(s)} → ${fmt(e)}`;
}

function setStatus(msg, color = '#aaa') {
  const el = document.getElementById('det-status-label');
  if (el) { el.textContent = msg; el.style.color = color; }
}

async function handleUpdate() {
  const settings = {
    enabled:    document.getElementById('det-enabled').checked,
    mode:       document.querySelector('input[name="det-mode"]:checked')?.value || 'time',
    time_start: parseInt(document.getElementById('det-time-start').value, 10),
    time_end:   parseInt(document.getElementById('det-time-end').value, 10),
    classes:    [...document.querySelectorAll('#det-class-grid input:checked')]
                  .map(el => el.dataset.class),
  };

  if (settings.classes.length === 0) {
    setStatus('⚠ Select at least 1 class', '#f0a500'); return;
  }

  setStatus('⏳ Sending...', '#aaa');
  document.getElementById('det-update-btn').disabled = true;

  try {
    const result = await window.electronAPI.updateDetection(settings);
    setStatus(
      result.success ? '✅ ' + (result.message || 'Updated') : '❌ ' + (result.message || 'Failed'),
      result.success ? '#00C851' : '#ff4444'
    );
  } catch (err) {
    setStatus('❌ ' + err.message, '#ff4444');
  } finally {
    document.getElementById('det-update-btn').disabled = false;
  }
}