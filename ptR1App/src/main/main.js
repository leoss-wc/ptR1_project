//main.js - Electron Main Process


const { app, BrowserWindow, ipcMain, dialog, protocol } = require('electron');
app.commandLine.appendSwitch('lang', 'en-GB');
const path = require('path');
const { Worker } = require('worker_threads');
const fs = require('fs');
const fsPromises = require('fs').promises;
const { spawn, exec } = require('child_process');
const yaml = require('js-yaml');
const { EventEmitter } = require('events'); 
const { initCaptureModule } = require('./captureModule');
const ffmpegPath = require('ffmpeg-static');


// --- Global Variables ---
let rosWorker;
let mainWindow;
let pythonProcess = null;
const internalEvents = new EventEmitter();
let captureHandlers = null; // ← ประกาศ module scope เพื่อให้ createRosWorker() เข้าถึงได้

// --- Paths & Constants ---
const userDataPath = app.getPath('userData');
const robotFilePath = path.join(userDataPath, 'robots.json');
const mapCacheDir = path.join(userDataPath, 'map_cache');
const mapFolder = path.join(userDataPath, 'maps');
const dataFolder = path.join(userDataPath, 'data');
const videoFolder = path.join(app.getPath('videos'), 'ptR1');
const captureDir = path.join(app.getPath('pictures'), 'ptR1_capture');


const isDev = !app.isPackaged;

// --- Dev Server for Videos ---
if (isDev) {
  const express = require('express');
  const serveStatic = require('serve-static');
  const appServer = express();
  // Quick Fix: Hardcoded path for dev
  appServer.use('/videos', serveStatic(videoFolder));
  appServer.listen(3001, () => {
    console.log('🎥 Video static server running on http://localhost:3001/videos');
  });
}

// ==========================================================
// Helper Functions
// ==========================================================

function loadRobotsFromFile() {
  try {
    if (fs.existsSync(robotFilePath)) return JSON.parse(fs.readFileSync(robotFilePath, 'utf-8'));
  } catch (e) { console.error('[ERROR] Failed to read robot file:', e); }
  return [];
}

function saveRobotsToFile(robots) {
  try {
    fs.writeFileSync(robotFilePath, JSON.stringify(robots, null, 2));
    return true;
  } catch (e) {
    console.error('[ERROR] Failed to save robot file:', e);
    return false;
  }
}

const getAllVideoFilesAsync = async (dirPath) => {
  let files = [];
  try {
    const stat = await fsPromises.stat(dirPath);
    if (!stat.isDirectory()) {
      console.warn(`getAllVideoFilesAsync: path is not a directory: ${dirPath}`);
      return [];
    }
    const items = await fsPromises.readdir(dirPath, { withFileTypes: true });
    for (const item of items) {
      const fullPath = path.join(dirPath, item.name);
      if (item.isDirectory()) {
        files = files.concat(await getAllVideoFilesAsync(fullPath));
      } else if (/\.(mp4|webm|mov)$/i.test(item.name)) {
        const stats = await fsPromises.stat(fullPath);
        files.push({
          path: fullPath,
          relativePath: path.relative(videoFolder, fullPath),
          name: item.name,
          mtime: stats.mtimeMs
        });
      }
    }
  } catch (err) { console.error("Error reading video directory:", err); }
  return files;
};

// ==========================================================
// ROS Worker Setup & Handling
// ==========================================================

function createRosWorker() {
  try {
    rosWorker = new Worker(path.join(__dirname, 'server.js'));

    rosWorker.on('message', (message) => {
      //Handle Responses (ส่งเข้า internalEvents เพื่อให้ IPC handle รอรับ)
      switch (message.type) {
        case 'select-map-response':
          internalEvents.emit('select-map-done', message.data);
          return;
        case 'nav-start-response':
          internalEvents.emit('nav-start-done', message.data);
          return;
        case 'nav-init-home-response':
          internalEvents.emit('nav-init-home-done', message.data);
          return;
        case 'map-save-edited':
            internalEvents.emit('map-save-edited-done', message.data);
            return;
        case 'startStreamResponse':
            internalEvents.emit('start-stream-done', message);
            return;
        case 'stopStreamResponse':
            internalEvents.emit('stop-stream-done', message);
            return;
        case 'detection-update-result':internalEvents.emit('detection-update-done', message.data);
            break;
        case 'home-result': 
            mainWindow.webContents.send('nav:home-result', message.data); 
            internalEvents.emit('home-result', message.data); 
            return;
        case 'capture-result':                          // ← ย้ายมาอยู่ใน switch แรก
            captureHandlers?.handleWorkerMessage(message);
            return;
      }

      //Handle Updates (ส่งตรงไปหน้าเว็บ)
      if (!mainWindow || mainWindow.isDestroyed()) return;

      switch (message.type) {
        // --- Status Updates ---
        case 'connection':
        const isConnected = message.data.isConnected;
        const isConnecting = message.data.isConnecting ?? false;
        mainWindow.webContents.send('connection-status', {
          connected: isConnected,
          connecting: isConnecting,
          message: isConnected ? 'Connected' : (isConnecting ? 'Connecting...' : 'Disconnected')
        });
        break;
        case 'log': console.log('Worker Log:', message.data); break;
        case 'error': console.error('Worker Error:', message.data); break;

        // --- Data Updates ---
        case 'tf-update': mainWindow.webContents.send('tf-update', message.data); break;
        case 'robot-pose-amcl': mainWindow.webContents.send('robot-pose-amcl', message.data); break;
        case 'robot-pose-slam': mainWindow.webContents.send('robot-pose-slam', message.data); break;
        case 'laser-scan-update': mainWindow.webContents.send('laser-scan-data', message.data); break;
        case 'live-map': mainWindow.webContents.send('live-map', message.data); break;
        case 'planned-path': mainWindow.webContents.send('planned-path', message.data); break;
        case 'stream-status': mainWindow.webContents.send('stream-status', message.data); break;
        case 'robot-status-update': mainWindow.webContents.send('robot-status', message.data); break;
        case 'system-profile-update': mainWindow.webContents.send('system-profile-update', message.data); break;
        case 'detection-alert':mainWindow?.webContents.send('detection-alert', message.data);break;
        
        
        // --- Map Operations ---
        case 'map-list': mainWindow.webContents.send('ros:map-list', message.data); break;
        case 'select-nav-map-result': mainWindow.webContents.send('select-nav-map', message.data); break;
        case 'map-base64': mainWindow.webContents.send('ros:map-base64', message.data); break;
        case 'map-save-result': mainWindow.webContents.send('map-save-result', message.data); break;
        
        // --- SLAM / Nav Results ---
        case 'slam-result': mainWindow.webContents.send('slam-result', message.data); break;
        case 'slam-stop-result': mainWindow.webContents.send('slam-stop-result', message.data); break;
        case 'slam-reset-result': mainWindow.webContents.send('slam-reset-result', message.data); break;
        case 'goal-result': 
            console.log('[Main] Forwarding goal result:', message.data);
            mainWindow.webContents.send('goal-result', message.data); 
            break;

        // --- Patrol Results ---
        case 'patrol-start-result': mainWindow.webContents.send('patrol-start-result', message.data); break;
        case 'patrol-pause-result': mainWindow.webContents.send('patrol-pause-result', message.data); break;
        case 'patrol-resume-result': mainWindow.webContents.send('patrol-resume-result', message.data); break;
        case 'patrol-stop-result': mainWindow.webContents.send('patrol-stop-result', message.data); break;
        case 'patrol-status': mainWindow.webContents.send('patrol-status', message.data); break;

        default:
          console.warn('[main]: Unknown message from worker:', message.type);
      }
    });

    rosWorker.on('error', (err) => console.error('[main]: Worker Error:', err));
    rosWorker.on('exit', (code) => console.log(`[main]: Worker exited with code ${code}`));

  } catch (error) {
    console.error('❌ Failed to create Worker:', error);
  }
}

// ==========================================================
// PC Handlers
// ==========================================================

// ---System & Window ---
ipcMain.handle('get-env-is-dev', () => isDev);
ipcMain.handle('get-userdata-path', (_, subfolder = '') => path.join(userDataPath, subfolder));
ipcMain.on('connect-rosbridge', (event, ip, port) => {
  const url = `ws://${ip}:${port || '9090'}`;
  rosWorker?.postMessage({ type: 'connectROS', url });
});

// ---Navigation (Request/Response Pattern) ---
ipcMain.handle('map-select', async (event, mapName) => {
    rosWorker?.postMessage({ type: 'selectNavMap', mapName }); // ต้องแก้ฝั่ง server.js ให้รับชื่อ type นี้
    
    return new Promise((resolve) => {
        const handler = (data) => {
            clearTimeout(timeout);
            resolve(data);
        };
        internalEvents.once('select-map-done', handler);
        const timeout = setTimeout(() => {
            internalEvents.off('select-map-done', handler);
            resolve({ success: false, message: "Timeout selecting map" });
        }, 10000);
    });
});

ipcMain.handle('nav-start', async (event, restorePose) => {
    rosWorker?.postMessage({ type: 'startNavigation', data: { restorePose } });
    
    return new Promise((resolve) => {
        const handler = (data) => {
            clearTimeout(timeout);
            resolve(data);
        };
        internalEvents.once('nav-start-done', handler);
        const timeout = setTimeout(() => {
            internalEvents.off('nav-start-done', handler);
            resolve({ success: false, message: "Timeout starting nav" });
        }, 15000);
    });
});

ipcMain.handle('nav-init-home', async (event, mapName) => {
    rosWorker?.postMessage({ type: 'initHome', mapName }); 
    return { success: true, message: "Init home command sent." }; 
});

ipcMain.handle('nav-stop', async (event, savePose) => {
    rosWorker?.postMessage({ type: 'stopNavigation', data: { savePose } });
    return { success: true, message: "Stop command sent." };
});

// ---  Map Management ---
ipcMain.on('sync-maps', async () => {
    // Logic การ Sync Map เดิม (ย่อไว้)
    const localMapFolder = path.join(mapFolder);
    const pngFolder = path.join(localMapFolder, 'png');
    const yamlFolder = path.join(localMapFolder, 'yaml');
    fs.mkdirSync(pngFolder, { recursive: true });
    fs.mkdirSync(yamlFolder, { recursive: true });

    const localMapFiles = fs.readdirSync(pngFolder).filter(f => f.endsWith('.png')).map(f => path.basename(f, '.png'));
    rosWorker.postMessage({ type: 'listMaps' });

    rosWorker.once('message', async (message) => {
        if (message.type !== 'map-list') return;
        const mapsToDownload = message.data.filter(name => !localMapFiles.includes(name));
        const imageArray = [];

        for (const name of mapsToDownload) {
            rosWorker.postMessage({ type: 'requestMapFileAsBase64', mapName: name });
            await new Promise((resolve) => {
                const handler = (msg) => {
                    if (msg.type === 'map-data' && msg.data.name === name) {
                        rosWorker.off('message', handler);
                        const buffer = Buffer.from(msg.data.base64, 'base64');
                        fs.writeFileSync(path.join(pngFolder, `${name}.png`), buffer);
                        fs.writeFileSync(path.join(yamlFolder, `${name}.yaml`), msg.data.yaml);
                        imageArray.push({ name, base64: `data:image/png;base64,${msg.data.base64}` });
                        resolve();
                    }
                };
                rosWorker.on('message', handler);
            });
        }
        mainWindow.webContents.send('sync-complete', imageArray);
    });
});

ipcMain.handle('get-local-maps', async () => {
    const pngFolder = path.join(mapFolder, 'png');
    if (!fs.existsSync(pngFolder)) return [];
    const files = fs.readdirSync(pngFolder).filter(f => f.endsWith('.png')).map(f => {
        const buffer = fs.readFileSync(path.join(pngFolder, f));
        return { name: path.basename(f, '.png'), base64: `data:image/png;base64,${buffer.toString('base64')}` };
    });
    return files.sort((a, b) => a.name.localeCompare(b.name)).reverse();
});

ipcMain.handle('save-edited-map', async (_, { newName, base64, yamlContent }) => {
    rosWorker?.postMessage({ type: 'saveEditedMap', data: { name: newName, base64, yamlContent } });
    return new Promise((resolve) => {
        internalEvents.once('map-save-edited-done', (data) => resolve(data));
        setTimeout(() => resolve({ success: false, message: "Timeout" }), 10000);
    });
});

ipcMain.on('delete-map', (_, mapName) => {
    try {
        // ลบไฟล์ Map Local (PNG & YAML)
        const pngPath = path.join(mapFolder, 'png', `${mapName}.png`);
        const yamlPath = path.join(mapFolder, 'yaml', `${mapName}.yaml`);
        
        if (fs.existsSync(pngPath)) fs.unlinkSync(pngPath);
        if (fs.existsSync(yamlPath)) fs.unlinkSync(yamlPath);

        // ลบไฟล์ Cache
        // mapCacheDir ต้องถูกประกาศไว้ด้านบนของไฟล์ main.js แล้ว (path.join(userDataPath, 'map_cache'))
        const cachePath = path.join(mapCacheDir, `${mapName}.json`);
        
        if (fs.existsSync(cachePath)) {
            fs.unlinkSync(cachePath);
            console.log(`[Main] Deleted map cache for: ${mapName}`);
        }
        rosWorker?.postMessage({ type: 'deleteMap', mapName });

    } catch (error) {
        console.error(`[Main] Error deleting map '${mapName}':`, error);
    }
});
ipcMain.handle('get-map-meta', async (_, mapName) => {
  const yamlPath = path.join(mapFolder, 'yaml', `${mapName}.yaml`);
  try {
    if (!fs.existsSync(yamlPath)) throw new Error("YAML file not found");
    const content = fs.readFileSync(yamlPath, 'utf8');
    const meta = yaml.load(content);
    return {
      success: true,
      data: { resolution: meta.resolution, origin: meta.origin, image: meta.image }
    };
  } catch (err) {
    return { success: false, message: err.message };
  }
});
ipcMain.handle('get-map-data-by-name', async (_, mapName) => {
  try {
    const pngPath = path.join(mapFolder, 'png', `${mapName}.png`);
    const yamlPath = path.join(mapFolder, 'yaml', `${mapName}.yaml`);

    if (!fs.existsSync(pngPath) || !fs.existsSync(yamlPath)) {
      throw new Error(`Map files for '${mapName}' not found.`);
    }

    const imageBuffer = fs.readFileSync(pngPath);
    const base64Data = `data:image/png;base64,${imageBuffer.toString('base64')}`;
    const yamlContent = fs.readFileSync(yamlPath, 'utf8');
    const metaData = yaml.load(yamlContent);

    return {
      success: true,
      name: mapName,
      base64: base64Data,
      meta: { resolution: metaData.resolution, origin: metaData.origin }
    };
  } catch (error) {
    console.error(`Error in getMapDataByName: ${error.message}`);
    return { success: false, message: error.message };
  }
});

// --- 3.4 Robot Control (Fire-and-forget) ---
ipcMain.on('robot-command', (_, command) => rosWorker?.postMessage({ type: 'sendCmd', command }));
ipcMain.on('twist-command', (_, data) => rosWorker?.postMessage({ type: 'sendTwist', data }));
ipcMain.on('ros:send-servo-tilt-int16', (_, angle) => rosWorker?.postMessage({ type: 'sendServoTiltInt16', angle }));
ipcMain.on('ros:send-servo-pan-int16', (_, angle) => rosWorker?.postMessage({ type: 'sendServoPanInt16', angle }));
ipcMain.on('set-manual-mode', (_, { state }) => rosWorker?.postMessage({ type: 'sendCmd', command: state ? 'manual_on' : 'manual_off' }));
ipcMain.on('uint32-command', (_, msg) => {
    const command = (msg.variableId & 0xFF) << 24 | (msg.value & 0xFFFFFF);
    rosWorker?.postMessage({ type: 'command', command });
});
ipcMain.on('relay-command', (_, data) => rosWorker?.postMessage({ type: 'sendRelay', ...data }));

// --- 3.5 Patrol & Goals ---
ipcMain.on('start-patrol', (_, data) => rosWorker?.postMessage({ type: 'startPatrol', ...data }));
ipcMain.on('pause-patrol', () => rosWorker?.postMessage({ type: 'pausePatrol' }));
ipcMain.on('resume-patrol', () => rosWorker?.postMessage({ type: 'resumePatrol' }));
ipcMain.on('stop-patrol', () => rosWorker?.postMessage({ type: 'stopPatrol' }));

// ---SLAM ---
ipcMain.on('start-slam', () => rosWorker?.postMessage({ type: 'startSLAM' }));
ipcMain.on('stop-slam', () => rosWorker?.postMessage({ type: 'stopSLAM' }));
ipcMain.on('reset-slam', () => rosWorker?.postMessage({ type: 'resetSLAM' }));
ipcMain.on('save-map', (_, mapName) => rosWorker?.postMessage({ type: 'saveMap', mapName }));

// --- Video & Files ---
ipcMain.handle('load:videos', async (_, customPath) => {
    const baseDir = customPath || videoFolder;
    if (!fs.existsSync(baseDir)) return [];
    const files = await getAllVideoFilesAsync(baseDir);
    return files.sort((a, b) => b.mtime - a.mtime);
});
ipcMain.handle('get-default-video-path', () => {
  return path.join(app.getPath('videos'), 'ptR1');
});
ipcMain.handle('get-video-path', (_, relativePath) => {
  // สร้าง full path ไปยังไฟล์วิดีโอ
  const fullPath = path.join(app.getPath('videos'), 'ptR1', relativePath);
  
  // แปลงเป็น Custom Protocol URL (video://...) เพื่อให้ HTML5 Video Player อ่านได้
  // ต้อง replace backslash (\) เป็น slash (/) สำหรับ Windows
  return `video://${fullPath.replace(/\\/g, '/')}`; 
});

ipcMain.handle('dialog:select-folder', async () => {
    const result = await dialog.showOpenDialog(mainWindow, { 
        properties: ['openFile'],
        title: 'Select Any Video in Folder',
        buttonLabel: 'Select',
        filters: [{ name: 'Videos', extensions: ['mp4', 'webm', 'mov'] }],
        defaultPath: app.getPath('videos'),
    });
    if (result.canceled) return null;

    // return folder ที่ไฟล์นั้นอยู่
    return path.dirname(result.filePaths[0]);
});

ipcMain.handle('save-video', async (_, { buffer, date, filename }) => {
    const baseDir = path.join(videoFolder, date);
    if (!fs.existsSync(baseDir)) fs.mkdirSync(baseDir, { recursive: true });

    const webmPath = path.join(baseDir, filename);
    const mp4Path  = webmPath.replace(/\.webm$/, '.mp4');

    await fsPromises.writeFile(webmPath, Buffer.from(buffer));

    return new Promise((resolve) => {
        exec(`"${ffmpegPath}" -y -i "${webmPath}" -c:v libx264 -c:a aac "${mp4Path}"`, (error) => {
            const success = !error;
            if (success) {
                fs.unlink(webmPath, () => {});
                console.log(`✅ Converted: ${mp4Path}`);
            } else {
                console.error('❌ FFmpeg Error:', error);
            }
            mainWindow?.webContents.send('video-save-status', {
                success,
                path: success ? mp4Path : null
            });
            resolve({ success });
        });
    });
});

ipcMain.handle('start-stream', async () => {
    rosWorker?.postMessage({ type: 'startStream' });
    return new Promise(resolve => {
        internalEvents.once('start-stream-done', (msg) => resolve(msg.success));
        setTimeout(() => resolve(false), 7000);
    });
});

ipcMain.handle('stop-stream', async () => {
    rosWorker?.postMessage({ type: 'stopStream' });
    return new Promise(resolve => {
        internalEvents.once('stop-stream-done', () => resolve(true));
        setTimeout(() => resolve(true), 3000);
    });
});
ipcMain.on('save-dataset-image', (event, base64Data) => {
    if (!fs.existsSync(captureDir)){
      fs.mkdirSync(captureDir);
    }
    // ตัดส่วนหัว 'data:image/jpeg;base64,' ออก
    const base64Image = base64Data.split(';base64,').pop();
    
    // ตั้งชื่อไฟล์ด้วย Timestamp
    const fileName = `frame_${Date.now()}.jpg`;
    const filePath = path.join(captureDir, fileName);

    // บันทึกไฟล์
    fs.writeFile(filePath, base64Image, {encoding: 'base64'}, function(err) {
        if (err) console.log('Error saving image:', err);
    });
});

ipcMain.handle('dialog:select-folder-map', async (_, defaultPath = null) => {
    const result = await dialog.showOpenDialog({
        properties: ['openDirectory'],
        defaultPath: defaultPath || undefined
    });
    if (result.canceled) return null;
    return result.filePaths[0];
});

// settings:save / settings:load
const settingsPath = path.join(userDataPath, 'settings.json');
ipcMain.handle('settings:save', async (_, settings) => {
    try {
        fs.writeFileSync(settingsPath, JSON.stringify(settings, null, 2));
        return { success: true };
    } catch (e) {
        return { success: false, message: e.message };
    }
});
ipcMain.handle('settings:load', async () => {
    try {
        if (!fs.existsSync(settingsPath)) return null;
        return JSON.parse(fs.readFileSync(settingsPath, 'utf-8'));
    } catch (e) {
        return null;
    }
});

ipcMain.handle('mapcache:delete', async (_, mapName) => {
    try {
        const cachePath = path.join(mapCacheDir, `${mapName}.json`);
        if (fs.existsSync(cachePath)) fs.unlinkSync(cachePath);
        return true;
    } catch (e) {
        console.error('[Main] mapcache:delete error:', e);
        return false;
    }
});



ipcMain.handle('nav:get-home', async (_, mapName) => {
  try {
    const homeConfigPath = path.join(dataFolder, 'map_homes.json');
    
    if (!fs.existsSync(homeConfigPath)) {
        return { success: false, message: "Home config file not found" };
    }

    const fileContent = fs.readFileSync(homeConfigPath, 'utf8');
    const homesData = JSON.parse(fileContent);

    if (homesData[mapName]) {
        return { success: true, data: homesData[mapName] };
    } else {
        return { success: false, message: `No home set for map '${mapName}'` };
    }

  } catch (error) {
    console.error("Error reading home config:", error);
    return { success: false, message: error.message };
  }
});
ipcMain.on('set-initial-pose', (_, pose) => {
    console.log('[Main] Setting Initial Pose:', pose);
    rosWorker?.postMessage({ 
        type: 'setInitialPose', 
        pose: pose 
    });
});
ipcMain.handle('nav:set-home', async (_, mapName) => {
    // 1. ส่งคำสั่งไปที่ Worker
    rosWorker?.postMessage({ type: 'setHome', mapName });

    // 2. สร้าง Promise รอผลลัพธ์
    return new Promise((resolve) => {
        const timeoutMs = 5000; // รอสูงสุด 5 วินาที
        // ฟังก์ชัน Callback เมื่อได้รับผล
        const handler = (data) => {
            // เช็คว่าเป็นผลลัพธ์ของ action 'Set Home' หรือไม่
            if (data.action === 'Set Home') {
                clearTimeout(timeoutTimer);
                internalEvents.off('home-result', handler); // ลบ Listener ออก
                resolve(data); // ส่งผลลัพธ์กลับไปให้ frontend
            }
        };

        // ตั้ง Timeout กันค้าง
        const timeoutTimer = setTimeout(() => {
            internalEvents.off('home-result', handler);
            resolve({ success: false, message: "Timeout: No response from ROS" });
        }, timeoutMs);
        // เริ่มรอฟัง Event
        internalEvents.on('home-result', handler);
    });
});

ipcMain.handle('nav:go-home', async (_, mapName) => {
    rosWorker?.postMessage({ type: 'goHome', mapName });

    return new Promise((resolve) => {
        const handler = (data) => {
            if (data.action === 'Go Home') {
                clearTimeout(timeoutTimer);
                internalEvents.off('home-result', handler);
                resolve(data);
            }
        };

        const timeoutTimer = setTimeout(() => {
            internalEvents.off('home-result', handler);
            resolve({ success: false, message: "Timeout: No response from ROS" });
        }, 600000); // 600s ตรงกับ HOME_TIMEOUT_MS ใน shutdownManager.js

        internalEvents.on('home-result', handler);
    });
});


//AI Detection Update
ipcMain.handle('detection:update', async (_, settings) => {
  rosWorker?.postMessage({ type: 'updateDetection', data: settings });

  return new Promise((resolve) => {
    internalEvents.once('detection-update-done', (data) => resolve(data));
    setTimeout(() => {
      resolve({ success: false, message: 'Timeout: No response from ROS' });
    }, 5000);
  });
});

// --- 3.8 Cache & Robots ---
ipcMain.handle('robots:load', loadRobotsFromFile);
ipcMain.handle('robots:save', (_, robots) => saveRobotsToFile(robots));
ipcMain.handle('mapcache:save', async (_, { mapName, imageData }) => {
    fs.mkdirSync(mapCacheDir, { recursive: true });
    await fs.promises.writeFile(path.join(mapCacheDir, `${mapName}.json`), JSON.stringify(imageData));
    return true;
});
ipcMain.handle('mapcache:load', async (_, mapName) => {
    try { return JSON.parse(await fs.promises.readFile(path.join(mapCacheDir, `${mapName}.json`), 'utf-8')); }
    catch { return null; }
});


// 4. App Lifecycle & Window Creation
function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1280,
    height: 720,
    icon: path.join(__dirname, '../../assets/icon.png'),
    webPreferences: {
      preload: path.join(__dirname, '/preload.js'),
      contextIsolation: true,
      nodeIntegration: false,
      sandbox: false,
      contentSecurityPolicy: `
        default-src 'self';
        script-src 'self';
        style-src 'self' 'unsafe-inline';
        img-src 'self' data: blob:;
        connect-src 'self' ws: http:;
        media-src 'self' blob: http: video:;
      `
    },
  });
  
  if(isDev) mainWindow.webContents.openDevTools();
  mainWindow.loadFile(path.join(__dirname, '../renderer/index.html'));
}

app.whenReady().then(() => {
  // Setup Protocols
  protocol.registerFileProtocol('video', (request, callback) => {
    const url = request.url.replace('video://', '');
    try { return callback(decodeURI(url)); } catch (error) { console.error(error); }
  });

  // Init Folders
  if (!fs.existsSync(mapFolder)) fs.mkdirSync(mapFolder, { recursive: true });
  if (!fs.existsSync(dataFolder)) {fs.mkdirSync(dataFolder, { recursive: true });}

  // Init Processes
  //startPythonBackend();
  createRosWorker();
  createWindow();
  captureHandlers = initCaptureModule(
    ipcMain,
    app,
    () => rosWorker,
    () => mainWindow
  );


  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

app.on('before-quit', async () => {
  if (rosWorker) {
    rosWorker.postMessage({ type: 'stopNavigation', data: { savePose: true } });
    rosWorker.postMessage({ type: 'stopStream' });
    await new Promise(r => setTimeout(r, 500));
    rosWorker.terminate();
  }
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit();
});