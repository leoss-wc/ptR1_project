/**
 * captureModule.js — Main Process
 * จัดการ IPC สำหรับ Dataset Capture
 *
 * วิธีใช้: เพิ่มใน main.js
 *   const { initCaptureModule } = require('./captureModule');
 *   initCaptureModule(ipcMain, app, rosWorker, mainWindow);  // เรียกหลัง createWindow()
 */

const fs   = require('fs');
const path = require('path');

/**
 * @param {Electron.IpcMain} ipcMain
 * @param {Electron.App}     app
 * @param {() => Worker}     getRosWorker   — ฟังก์ชันคืน rosWorker (เพราะ worker อาจ recreate)
 * @param {() => BrowserWindow} getMainWindow — ฟังก์ชันคืน mainWindow
 */
function initCaptureModule(ipcMain, app, getRosWorker, getMainWindow) {

  // ---- Path ที่เก็บภาพบน PC (Electron Host) ----------------------------
  // ภาพ RAW จาก Raspi จะถูกเก็บไว้ที่ ~/Pictures/ptR1_dataset/<label>/
  const datasetBaseDir = path.join(app.getPath('pictures'), 'ptR1_dataset');

  // ---- Helper -----------------------------------------------------------
  function ensureDir(dirPath) {
    if (!fs.existsSync(dirPath)) fs.mkdirSync(dirPath, { recursive: true });
  }

  // ---- IPC: Capture Single Shot -----------------------------------------
  // Frontend → ipcMain → rosWorker → ROS Service /stream_manager/capture
  ipcMain.handle('capture:single', async (_, label = 'object') => {
    const worker = getRosWorker();
    if (!worker) return { success: false, message: 'ROS worker not ready' };
    worker.postMessage({ type: 'captureSnapshot', label });
    return { success: true };
  });

  // ---- IPC: Start Burst Capture -----------------------------------------
  // Frontend → ipcMain → rosWorker → ROS Topic /stream_manager/capture_config
  ipcMain.handle('capture:startBurst', async (_, { label = 'object', interval = 2.0 }) => {
    const worker = getRosWorker();
    if (!worker) return { success: false, message: 'ROS worker not ready' };
    worker.postMessage({ type: 'captureBurstStart', label, interval });
    return { success: true };
  });

  // ---- IPC: Stop Burst Capture ------------------------------------------
  ipcMain.handle('capture:stopBurst', async () => {
    const worker = getRosWorker();
    if (!worker) return { success: false, message: 'ROS worker not ready' };
    worker.postMessage({ type: 'captureBurstStop' });
    return { success: true };
  });

  // ---- IPC: Open dataset folder in File Explorer -----------------------
  ipcMain.handle('capture:openFolder', async (_, label = '') => {
    const { shell } = require('electron');
    const targetDir = label
      ? path.join(datasetBaseDir, label)
      : datasetBaseDir;
    ensureDir(targetDir);
    shell.openPath(targetDir);
    return { success: true, path: targetDir };
  });

  // ---- IPC: Get capture stats (count per label) ------------------------
  ipcMain.handle('capture:getStats', async () => {
    try {
      ensureDir(datasetBaseDir);
      const labels = fs.readdirSync(datasetBaseDir, { withFileTypes: true })
        .filter(d => d.isDirectory())
        .map(d => {
          const labelDir = path.join(datasetBaseDir, d.name);
          const count = fs.readdirSync(labelDir).filter(f => f.endsWith('.jpg')).length;
          return { label: d.name, count };
        });
      return { success: true, labels, baseDir: datasetBaseDir };
    } catch (e) {
      return { success: false, labels: [], message: e.message };
    }
  });

  // ---- Listen: ROS Worker → 'captured' event ---------------------------
  // rosWorker จะส่ง type: 'capture-result' มาเมื่อ Raspi บันทึกภาพสำเร็จ
  // payload: { filename, label, count }
  // Main process relay ไปยัง renderer
  function handleWorkerMessage(message) {
    const win = getMainWindow();
    if (!win || win.isDestroyed()) return;

    if (message.type === 'capture-result') {
      win.webContents.send('capture:result', message.data);
    }
  }

  // ---- Export helper เพื่อให้ main.js ติด listener ได้ -----------------
  // เรียกใช้ใน rosWorker.on('message', ...) ที่มีอยู่แล้ว
  return { handleWorkerMessage };
}

module.exports = { initCaptureModule };
