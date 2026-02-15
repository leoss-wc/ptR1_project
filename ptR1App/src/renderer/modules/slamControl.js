// modules/slamControl.js
export function initSlamControl() {
    const startSlamBtn = document.getElementById('start-slam-btn');
    const stopSlamBtn = document.getElementById('stop-slam-btn');
    const saveMapBtn = document.getElementById('save-map-btn');
    const mapNameInput = document.getElementById('map-name-input');
    const slamResultLabel = document.getElementById('slam-result-label');
    const resetSlamBtn = document.getElementById('reset-slam-btn');


    const staticMapTab = document.getElementById('btn-static-map');
    const liveMapTab = document.getElementById('btn-live-map');


    function setSlamUIState(isScanning) {
        // ล็อคปุ่ม Start/Stop ตามสถานะ
        startSlamBtn.disabled = isScanning;
        stopSlamBtn.disabled = !isScanning;
        saveMapBtn.disabled = !isScanning; // จะเซฟได้ก็ต่อเมื่อกำลัง SLAM (หรือหยุดแล้วแต่ยังไม่เคลียร์)
        if (resetSlamBtn) resetSlamBtn.disabled = !isScanning;
        
        // ล็อคการเปลี่ยนแท็บ! (ห้ามหนีไปหน้า Static)
        if (staticMapTab) staticMapTab.disabled = isScanning;
        if (liveMapTab) liveMapTab.disabled = isScanning;

        // เปลี่ยนสีปุ่มให้รู้ว่ากดไม่ได้ 
        if (isScanning) {
            staticMapTab.style.opacity = '0.5';
            staticMapTab.style.cursor = 'not-allowed';
            startSlamBtn.classList.add('disabled');
        } else {
            staticMapTab.style.opacity = '1';
            staticMapTab.style.cursor = 'pointer';
            startSlamBtn.classList.remove('disabled');
        }
    }
    // ตั้งค่าเริ่มต้น: ยังไม่ SLAM -> ปุ่ม Stop กดไม่ได้
    setSlamUIState(false);

    startSlamBtn.addEventListener('click', () => {
        slamResultLabel.textContent = 'Starting SLAM...';
        slamResultLabel.style.color = 'yellow';
        setSlamUIState(true);
        //หยุด acml move_base mapserver ก่อนเริ่ม SLAM เพื่อป้องกันการรบกวนรวมอยยู่ใน api นี้แล้ว   
        window.electronAPI.startSLAM();
    });
    stopSlamBtn.addEventListener('click', () => {
        setSlamUIState(false);
        slamResultLabel.textContent = 'Stopping SLAM...';
        slamResultLabel.style.color = 'yellow';
        window.electronAPI.stopSLAM();
    });
    if (resetSlamBtn) {
        resetSlamBtn.addEventListener('click', () => {
            if (!confirm("⚠️ Are you sure you want to RESET the map? Current progress will be lost.")) return;

            slamResultLabel.textContent = 'Resetting SLAM...';
            slamResultLabel.style.color = 'yellow';
            window.electronAPI.resetSLAM();
        });
    }
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

    window.electronAPI.onSLAMStartResult((data) => {
        updateLabel(data);
        if (!data.success) {
            // ถ้า Start ไม่สำเร็จ -> ให้ปลดล็อคกลับมาเหมือนเดิม
            setSlamUIState(false);
            alert("Failed to start SLAM: " + data.message);
        }
    });

    window.electronAPI.onSLAMStopResult((data) => updateLabel(data));
    window.electronAPI.onSLAMResetResult((data) => {
        updateLabel(data);
        if (data.success) {
            console.log("SLAM Reset Successful");
        }
    });
    
    window.electronAPI.onMapSaveResult((result) => {
        updateLabel({success: result.success, message: `Save '${result.name}': ${result.message}`});
        if (result.success) {
            alert(`Map "${result.name}" saved! Syncing...`);
            document.getElementById('sync-maps-btn').click(); 
        }
    });
    console.log('Slam Control: Initialized SLAM control buttons and callbacks.');
}

function updateLabel(data) {
    const label = document.getElementById('slam-result-label');
    label.textContent = data.message;
    label.style.color = data.success ? 'lime' : 'red';
}