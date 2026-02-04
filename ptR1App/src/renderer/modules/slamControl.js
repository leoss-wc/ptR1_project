// modules/slamControl.js

export function initSlamControl() {
    const startSlamBtn = document.getElementById('start-slam-btn');
    const stopSlamBtn = document.getElementById('stop-slam-btn');
    const saveMapBtn = document.getElementById('save-map-btn');
    const mapNameInput = document.getElementById('map-name-input');
    const slamResultLabel = document.getElementById('slam-result-label');

    startSlamBtn.addEventListener('click', () => {
        slamResultLabel.textContent = 'Starting SLAM...';
        slamResultLabel.style.color = 'yellow';
        window.electronAPI.startSLAM();
    });

    stopSlamBtn.addEventListener('click', () => {
        slamResultLabel.textContent = 'Stopping SLAM...';
        slamResultLabel.style.color = 'yellow';
        window.electronAPI.stopSLAM();
    });

    saveMapBtn.addEventListener('click', () => {
        const mapName = mapNameInput.value.trim();
        if (!mapName) {
            alert('Please enter a map name.');
            return;
        }
        slamResultLabel.textContent = `Saving map: ${mapName}...`;
        slamResultLabel.style.color = 'yellow';
        window.electronAPI.saveMap(mapName);
    });

    // Callbacks
    window.electronAPI.onSLAMStartResult((data) => updateLabel(data));
    window.electronAPI.onSLAMStopResult((data) => updateLabel(data));
    
    window.electronAPI.onMapSaveResult((result) => {
        updateLabel({success: result.success, message: `Save '${result.name}': ${result.message}`});
        if (result.success) {
            alert(`Map "${result.name}" saved! Syncing...`);
            document.getElementById('sync-maps-btn').click(); 
        }
    });
}

function updateLabel(data) {
    const label = document.getElementById('slam-result-label');
    label.textContent = data.message;
    label.style.color = data.success ? 'lime' : 'red';
}