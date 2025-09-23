/** main.js (src/main/main.js) - ปรับให้รองรับ dev/build */
const { app, BrowserWindow, ipcMain, dialog } = require('electron');
const path = require('path');
const { Worker } = require('worker_threads');
const fs = require('fs');
const { exec } = require('child_process');
const { spawn } = require('child_process');
const yaml = require('js-yaml');

let rosWorker;
let mainWindow;
let pythonProcess;
const robotFilePath = path.join(app.getPath('userData'), 'robots.json');
const settingsFilePath = path.join(app.getPath('userData'), 'settings.json');



const isDev = !app.isPackaged;
// อ่าน IP Address จาก Environment Variable


// Quick Fix ชั่วคราวตอน dev ทำ proxy ไฟล์ผ่าน express หรือ http server
if (isDev) {
  const express = require('express');
  const serveStatic = require('serve-static');
  const appServer = express();
  appServer.use('/videos', serveStatic('/home/leoss/Videos/ptR1'));

  appServer.listen(3001, () => {
    console.log('🎥 Video static server running on http://localhost:3001/videos');
  });
}


function loadRobotsFromFile() {
  try {
    if (fs.existsSync(robotFilePath)) {
      return JSON.parse(fs.readFileSync(robotFilePath, 'utf-8'));
    }
  } catch (e) {
    console.error('[ERROR] Failed to read robot file:', e);
  }
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

ipcMain.handle('robots:load', () => {
  return loadRobotsFromFile();
});

ipcMain.handle('robots:save', (_, robots) => {
  return saveRobotsToFile(robots);
});


ipcMain.on('key-command', (_, { command }) => {
  if (rosWorker) {
    rosWorker.postMessage({ type: 'sendDrive', command: command });
  } else {
    console.error('❌ Worker not initialized when sending key-command');
  }
});

ipcMain.on('servo-command', (_, { command }) => {
  if (rosWorker) {
    rosWorker.postMessage({ type: 'sendServo', command: command });
  } else {
    console.error('❌ Worker not initialized when sending servo-command');
  }
});

ipcMain.on('uint32-command', (_, { command }) => {
  if (!rosWorker) {
    console.error('❌ Worker not initialized when sending uint32-command');
    return;
  }
  console.log(`🎮 Received uint32 Command: ${command} (main log)`);
  rosWorker.postMessage({ type: 'sendCmd', command: command });
});

ipcMain.handle('get-ws-port', async () => {
  return 8080;
});

ipcMain.handle('get-default-video-path', () => {
  // app.getPath('videos') จะได้ /home/leoss/Videos (หรือตาม username)
  return path.join(app.getPath('videos'), 'ptR1');
});

ipcMain.handle('dialog:select-folder', async () => {
  console.log('--- Handler with Workaround Running ---');

  const result = await dialog.showOpenDialog({
    // เรายังคงขอ 'openDirectory' ตามทฤษฎี
    properties: ['openDirectory'] 
  });

  console.log('Original dialog result:', result);

  if (result.canceled) {
    return null;
  }

  // --- นี่คือส่วนที่แก้ไข ---
  // 1. ดึง "ไฟล์" ที่มันคืนค่ามา
  const filePath = result.filePaths[0]; 

  // 2. ใช้ path.dirname() เพื่อหา "โฟลเดอร์" ที่ไฟล์นี้อยู่
  const dirPath = path.dirname(filePath); 

  console.log(`Workaround Applied: Dialog gave file (${filePath}), We are using dir (${dirPath})`);

  // 3. คืนค่า "โฟลเดอร์" กลับไปแทน
  return dirPath;
});

const getAllVideoFiles = (dirPath, arrayOfFiles = []) => {
  const files = fs.readdirSync(dirPath);
  files.forEach((file) => {
    const fullPath = path.join(dirPath, file);
    const stat = fs.statSync(fullPath);
    if (stat.isDirectory()) {
      getAllVideoFiles(fullPath, arrayOfFiles);
    } else if (/\.(mp4|webm|mov)$/i.test(file)) {
      arrayOfFiles.push({
        path: fullPath,
        relativePath: path.relative(path.join(app.getPath('videos'), 'ptR1'), fullPath),
        name: file,
        mtime: stat.mtimeMs
      });
    }
  });
  return arrayOfFiles;
};

ipcMain.handle('load:videos', async (event, customPath = null) => {
  const baseDir = customPath || path.join(app.getPath('videos'), 'ptR1');
  console.log('Loading videos from:', baseDir);
  if (!fs.existsSync(baseDir)) return [];

  const allVideos = getAllVideoFiles(baseDir);
  allVideos.sort((a, b) => b.mtime - a.mtime); // เรียงจากใหม่ → เก่า

  return allVideos;
});

ipcMain.on('relay-command', (_, { relayId, command }) => {
  if (!rosWorker) {
    console.error('❌ Worker not initialized when sending relay-command');
    return;
  }
  console.log(`🔧 Received relay command: ${relayId} → ${command}`);
  rosWorker.postMessage({
    type: 'sendRelay',
    relayId,
    command
  });
});

ipcMain.on('set-manual-mode', (event, { state }) => {
  if (state) {
    const command = 0x05000001; // เปิด MANUAL
    console.log(`Main: Switching MANUAL MODE ON → Send: 0x${command.toString(16)}`);
    rosWorker.postMessage({ type: 'sendCmd', command });
  } else {
    console.log('Main: Switching MANUAL MODE OFF → (no command sent)');
    // ไม่ส่งอะไร
  }
});

ipcMain.on('save-video', (event, { buffer, date, filename }) => {
  const baseDir = path.join(app.getPath('videos'), 'ptR1', date);
  if (!fs.existsSync(baseDir)) {
    fs.mkdirSync(baseDir, { recursive: true });
  }

  const webmPath = path.join(baseDir, filename);
  const mp4Path = webmPath.replace(/\.webm$/, '.mp4');

  // เขียนไฟล์ .webm
  fs.writeFile(webmPath, buffer, (err) => {
    if (err) {
      console.error(`❌ Write .webm failed: ${err}`);
      return;
    }
  
    // ✅ หลังเขียนเสร็จแน่นอน ค่อยแปลง
    const cmd = `ffmpeg -y -i "${webmPath}" -c:v libx264 -c:a aac "${mp4Path}"`;
    exec(cmd, (error, stdout, stderr) => {
      if (error) {
        console.error(`❌ FFmpeg error: ${error.message}`);
        console.error(stderr); // 🟡 ดูรายละเอียดเพิ่ม
        return;
      }
      console.log(`Saved MP4: ${mp4Path}`);
    });
  });
});

app.on('before-quit', () => {
  if (rosWorker) {
    rosWorker.terminate();
  }
});

app.on('window-all-closed', function () {
  if (process.platform !== 'darwin') app.quit();
});

ipcMain.on('connect-rosbridge', (event, ip) => {
  const url = `ws://${ip}:9090`;
  rosWorker.postMessage({ type: 'connectROS', url: url });
  console.log(`Main: 🔌 Connecting to ROSBridge at ${url}`);
});

ipcMain.on('sync-maps', async () => {
  const localMapFolder = path.join(app.getPath('userData'), 'maps');

  //กำหนด path ไปยังโฟลเดอร์ png และ yaml แล้วสร้างโฟลเดอร์ถ้ายังไม่มี
  const pngFolder = path.join(localMapFolder, 'png');
  const yamlFolder = path.join(localMapFolder, 'yaml');
  fs.mkdirSync(pngFolder, { recursive: true });
  fs.mkdirSync(yamlFolder, { recursive: true });

  //กลับไปเช็คไฟล์ .png ที่มีอยู่จากในโฟลเดอร์ png
  const localMapFiles = fs.readdirSync(pngFolder)
    .filter(file => file.endsWith('.png'))
    .map(file => path.basename(file, '.png'));

  rosWorker.postMessage({ type: 'listMaps' });

  rosWorker.once('message', async (message) => {
    if (message.type !== 'map-list') return;

    const rosMapList = message.data;
    const mapsToDownload = rosMapList.filter(name => !localMapFiles.includes(name));

    const imageArray = [];
    const pendingMaps = [];

    for (const name of mapsToDownload) {
      rosWorker.postMessage({ type: 'requestMapFileAsBase64', mapName: name });

      await new Promise((resolve) => {
        rosWorker.once('message', (msg) => {
          if (msg.type === 'map-data' && msg.data.name === name) {
            
            // ✅ 3. เปลี่ยน path ที่จะบันทึกไฟล์ให้ไปที่โฟลเดอร์ของตัวเอง
            
            // บันทึกไฟล์ .png ลงในโฟลเดอร์ png/
            const buffer = Buffer.from(msg.data.base64, 'base64');
            const pngFilePath = path.join(pngFolder, `${msg.data.name}.png`);
            fs.writeFileSync(pngFilePath, buffer);

            // บันทึกไฟล์ .yaml ลงในโฟลเดอร์ yaml/
            const yamlFilePath = path.join(yamlFolder, `${msg.data.name}.yaml`);
            fs.writeFileSync(yamlFilePath, msg.data.yaml);

            imageArray.push({
              name: msg.data.name,
              base64: `data:image/png;base64,${msg.data.base64}`
            });
            pendingMaps.push(msg.data.name);
            resolve();
          }
        });
      });
    }

    console.log(`✅ Synced maps: ${pendingMaps.join(', ')}`);
    mainWindow.webContents.send('sync-complete', imageArray);
  });
});

ipcMain.on('select-map', (event, mapName) => {
  rosWorker.postMessage({ type: 'loadMap', mapName });
});

ipcMain.on('save-map', (event, mapName) => {
  rosWorker.postMessage({ type: 'saveMap', mapName });
});

ipcMain.on('send-patrol-path', (_, pathArray) => {
  if (rosWorker) {
    rosWorker.postMessage({ type: 'sendPatrolPath', path: pathArray });
  }
});

ipcMain.on('send-stop-patrol', () => {
  if (rosWorker) {
    rosWorker.postMessage({ type: 'sendStopPatrol' });
  }
});

ipcMain.on('send-single-goal', (_, point) => {
  if (rosWorker) {
    rosWorker.postMessage({ type: 'sendSingleGoal', point });
  }
});

ipcMain.on('start-slam', () => {
  if (rosWorker) {
    rosWorker.postMessage({ type: 'startSLAM' });
  }
});

ipcMain.on('stop-slam', () => {
  if (rosWorker) rosWorker.postMessage({ type: 'stopSLAM' });
});

ipcMain.on('resume-patrol', (_, { path, index }) => {
  rosWorker.postMessage({ type: 'resumePatrol', path, index });
});

ipcMain.handle('get-map-meta', async (_, mapName) => {
  const mapFolder = path.join(app.getPath('userData'), 'maps','yaml');
  const yamlPath = path.join(mapFolder, `${mapName}.yaml`);

  try {
    if (!fs.existsSync(yamlPath)) {
      throw new Error("YAML file not found");
    }

    const content = fs.readFileSync(yamlPath, 'utf8');
    const meta = yaml.load(content);

    return {
      success: true,
      data: {
        resolution: meta.resolution,
        origin: meta.origin,
        image: meta.image
      }
    };
  } catch (err) {
    return {
      success: false,
      message: err.message
    };
  }
});

ipcMain.handle('get-map-data-by-name', async (event, mapName) => {
  try {
    const mapsFolder = path.join(app.getPath('userData'), 'maps');
    const pngFilePath = path.join(mapsFolder, 'png', `${mapName}.png`);
    const yamlFilePath = path.join(mapsFolder, 'yaml', `${mapName}.yaml`);

    // ตรวจสอบว่าไฟล์มีอยู่จริง
    if (!fs.existsSync(pngFilePath) || !fs.existsSync(yamlFilePath)) {
      throw new Error(`Map files for '${mapName}' not found.`);
    }

    // อ่านไฟล์ PNG แล้วแปลงเป็น Base64
    const imageBuffer = fs.readFileSync(pngFilePath);
    const base64Data = `data:image/png;base64,${imageBuffer.toString('base64')}`;
    
    // อ่านและ parse ไฟล์ YAML
    const yamlContent = fs.readFileSync(yamlFilePath, 'utf8');
    const metaData = yaml.load(yamlContent);

    // ส่งข้อมูลทั้งหมดกลับไป
    return {
      success: true,
      name: mapName,
      base64: base64Data,
      meta: {
        resolution: metaData.resolution,
        origin: metaData.origin,
      }
    };
  } catch (error) {
    console.error(`Error in getMapDataByName: ${error.message}`);
    return { success: false, message: error.message };
  }
});

ipcMain.handle('get-local-maps', async () => {
  const pngFolder = path.join(app.getPath('userData'), 'maps', 'png');

  // เช็คว่าโฟลเดอร์ png มีอยู่จริงหรือไม่
  if (!fs.existsSync(pngFolder)) {
    return [];
  }

  // อ่านไฟล์ทั้งหมดจากโฟลเดอร์ png
  const files = fs.readdirSync(pngFolder)
    .filter(file => file.endsWith('.png'))
    .map(file => {
      // สร้าง fullPath จากโฟลเดอร์ png
      const fullPath = path.join(pngFolder, file);
      const buffer = fs.readFileSync(fullPath);
      return {
        name: path.basename(file, '.png'),
        base64: `data:image/png;base64,${buffer.toString('base64')}`
      };
    });

  // เรียงลำดับไฟล์ตามชื่อ
  return files.sort((a, b) => a.name.localeCompare(b.name)).reverse();
});

ipcMain.handle('get-userdata-path', (_, subfolder = '') => {
  return path.join(app.getPath('userData'), subfolder);
});

ipcMain.handle('dialog:select-folder-map', async (event, defaultPath = null) => {
  const result = await dialog.showOpenDialog({
  properties: ['openDirectory'],
    defaultPath: defaultPath || app.getPath('home')  // ถ้าไม่ส่ง defaultPath, ใช้ home
  });
  return result.canceled ? null : result.filePaths[0];
});

ipcMain.handle('get-env-is-dev', () => !app.isPackaged);

ipcMain.handle('get-video-path', (_, relativePath) => {
  const fullPath = path.join(app.getPath('videos'), 'ptR1', relativePath);
  return `file://${fullPath.replace(/\\/g, '/')}`;
});

// 🔁 ส่งต่อ status จาก worker → renderer
function sendPatrolStatus(isMoving) {
  if (mainWindow) {
    mainWindow.webContents.send('patrol-status', isMoving);
  }
}

function createWindow(ip) {
  const wsURL = `ws://${ip}:8181`;
  const mediaURL = `http://${"127.0.0.1"}:3001`;
  // รัน backend YOLO
  const backendPath = path.join(__dirname, '../../../yoloBackend/app.py');
  pythonProcess = spawn('python3', [backendPath], {
    stdio: 'inherit'
  });

  mainWindow = new BrowserWindow({
    width: 1280,
    height: 720,
    icon: path.join(__dirname, '../../assets/icon.png'),
    webPreferences: {
      preload: path.join(__dirname, '../../preload/preload.js'),
      contextIsolation: true,
      nodeIntegration: false,
       sandbox: false,   

      contentSecurityPolicy: `
        default-src 'self';
        script-src 'self';
        style-src 'self' 'unsafe-inline';
        img-src 'self' data: blob:;
        connect-src 'self' ws: http:;
        media-src 'self' blob: http:;
      `
    
    },
  });
  mainWindow.webContents.openDevTools();
  mainWindow.loadFile(path.join(__dirname, '../renderer/index.html'));
}

app.whenReady().then(() => {

  const mapFolder = path.join(app.getPath('userData'), 'maps');
  if (!fs.existsSync(mapFolder)) {
    fs.mkdirSync(mapFolder, { recursive: true });
    console.log('📂 Created userData/maps folder:', mapFolder);
  } else {
    console.log('✅ userData/maps already exists:', mapFolder);
  }

  createWindow();

  try {
    rosWorker = new Worker(path.join(__dirname, 'server.js'));

    rosWorker.on('message', (message) => {
      switch (message.type) {
        case 'power':
          mainWindow?.webContents.send('power', message.data);
          break;
        case 'log':
          console.log('Worker Log:', message.data);
          break;
        case 'error':
          console.error('Worker Error:', message.data);
          break;
        case 'connection':
          mainWindow?.webContents.send('connection-status', message.data);
          break;
        case 'map-list':
          mainWindow.webContents.send('ros:map-list', message.data);
          break;
        case 'map-load':
          mainWindow.webContents.send('ros:map-load', message.data);
          break;
        case 'map-save':
          mainWindow.webContents.send('ros:map-save', message.data);
          break;
        case 'map-base64':
          mainWindow.webContents.send('ros:map-base64', message.data);
          break;
        case 'map-save-result':
          mainWindow.webContents.send('map-save-result', message.data);
          break;
        case 'patrol-status':
          sendPatrolStatus(message.data);
          break;
        case 'slam-result':
          mainWindow?.webContents.send('slam-result', message.data);
          break;
        case 'slam-stop-result':
          mainWindow?.webContents.send('slam-stop-result', message.data);
          break;
        case 'live-map':
          mainWindow?.webContents.send('live-map', message.data);
          break;
        case 'robot-pose':
          mainWindow?.webContents.send('robot-pose', message.data);
          break;
        case 'planned-path':
          mainWindow?.webContents.send('planned-path', message.data);
          break;

        default:
          console.warn('Unknown message from worker:', message);
      }
    });
  
    rosWorker.on('error', (error) => {
      console.error('❌ Worker Error:', error);
    });
  
    rosWorker.on('exit', (code) => {
      console.log(`🛑 Worker exited with code ${code}`);
    });
  
    ipcMain.on('uint32-command', (event, message) => {
    const variableId = message.variableId & 0xFF;
    const value = message.value & 0xFFFFFF;
    const command = (variableId << 24) | value;

    console.log(`📦 sendCommand: ID=${variableId}, Value=${value}, UInt32=0x${command.toString(16)}`);

    rosWorker.postMessage({ type: 'command', command });
  });

    //rosWorker.postMessage({ type: 'connectROS', url: 'ws://127.0.0.1:9090' });
    //rosWorker.postMessage({ type: 'startWSS', port: 8080 });
  } catch (error) {
    console.error('❌ Failed to create Worker:', error);
  }
  app.on('activate', function () {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});