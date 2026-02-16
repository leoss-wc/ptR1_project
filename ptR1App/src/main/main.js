/** main.js (src/main/main.js) - ปรับให้รองรับ dev/build */
const { app, BrowserWindow, ipcMain, dialog } = require('electron');
const path = require('path');
const { Worker } = require('worker_threads');
const fs = require('fs');
const fsPromises = require('fs').promises;
const { exec } = require('child_process');
const { spawn } = require('child_process');
const yaml = require('js-yaml');


let rosWorker;
let mainWindow;
let pythonProcess = null;
const robotFilePath = path.join(app.getPath('userData'), 'robots.json');
const settingsFilePath = path.join(app.getPath('userData'), 'settings.json');
const mapCacheDir = path.join(app.getPath('userData'), 'map_cache');
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

function startPythonBackend() {
  // ตรวจสอบ Platform เพื่อเลือก path ของ python (เผื่อใช้ Windows/Linux)
  const isWin = process.platform === 'win32';
  
  // Path ไปยัง Python Executable ใน venv
  // หมายเหตุ: เช็ค path ให้ตรงกับเครื่องจริงของคุณ (bin หรือ Scripts)
  const pythonExecutable = path.join(__dirname, '../../python-backend/venv/' + (isWin ? 'Scripts/python.exe' : 'bin/python'));
  
  // Path ไปยังไฟล์ yolo_app.py
  const backendPath = path.join(__dirname, '../../python-backend/yolo_app.py');

  console.log('[Main] 🐍 Starting Python Backend...');
  console.log(`[Main] Python Path: ${pythonExecutable}`);
  console.log(`[Main] Script Path: ${backendPath}`);

  if (fs.existsSync(pythonExecutable) && fs.existsSync(backendPath)) {
    // spawn process
    pythonProcess = spawn(pythonExecutable, [backendPath]);

    // รับค่า Console Log จาก Python มาแสดงใน Terminal ของ Electron
    pythonProcess.stdout.on('data', (data) => {
      console.log(`[Python]: ${data.toString().trim()}`);
    });

    pythonProcess.stderr.on('data', (data) => {
      console.error(`[Python Error]: ${data.toString().trim()}`);
    });

    pythonProcess.on('close', (code) => {
      console.log(`[Python] Backend exited with code ${code}`);
    });
  } else {
    console.error('[Main] ❌ Python executable or script not found!');
  }
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

const getAllVideoFilesAsync = async (dirPath) => {
  let files = [];
  try {
    const items = await fsPromises.readdir(dirPath, { withFileTypes: true });
    
    for (const item of items) {
      const fullPath = path.join(dirPath, item.name);
      if (item.isDirectory()) {
        const subFiles = await getAllVideoFilesAsync(fullPath);
        files = files.concat(subFiles);
      } else if (/\.(mp4|webm|mov)$/i.test(item.name)) {
        const stats = await fsPromises.stat(fullPath);
        files.push({
          path: fullPath,
          relativePath: path.relative(path.join(app.getPath('videos'), 'ptR1'), fullPath),
          name: item.name,
          mtime: stats.mtimeMs
        });
      }
    }
  } catch (err) {
    console.error("Error reading video directory:", err);
  }
  return files;
};


ipcMain.on('set-initial-pose', (_, pose) => {
  if (rosWorker) {
    console.log('[Main] Received initial pose, forwarding to ROS worker:', pose);
    rosWorker.postMessage({ type: 'setInitialPose', pose: pose });
  }
});

ipcMain.handle('robots:load', () => {
  return loadRobotsFromFile();
});

ipcMain.handle('robots:save', (_, robots) => {
  return saveRobotsToFile(robots);
});

ipcMain.handle('mapcache:save', async (_, { mapName, imageData }) => {
  try {
    if (!fs.existsSync(mapCacheDir)) {
      fs.mkdirSync(mapCacheDir);
    }
    const filePath = path.join(mapCacheDir, `${mapName}.json`);
    await fs.promises.writeFile(filePath, JSON.stringify(imageData));
    
    console.log(`[Cache] Saved processed data for map: ${mapName}`);
    return true;
  } catch (error) {
    console.error(`[Cache] Failed to save cache for map: ${mapName}`, error);
    return false;
  }
});

ipcMain.handle('mapcache:load', async (_, mapName) => {
  try {
    const filePath = path.join(mapCacheDir, `${mapName}.json`);
    if (fs.existsSync(filePath)) {
      const fileContent = await fs.promises.readFile(filePath, 'utf-8');
      console.log(`[Cache] Loaded processed data for map: ${mapName}`);
      return JSON.parse(fileContent);
    }
    return null; // ไม่พบไฟล์ cache
  } catch (error) {
    console.error(`[Cache] Failed to load cache for map: ${mapName}`, error);
    return null;
  }
});

ipcMain.handle('mapcache:delete', async (_, mapName) => {
  try {
    const filePath = path.join(mapCacheDir, `${mapName}.json`);
    // ตรวจสอบก่อนว่ามีไฟล์ไหม (เพื่อความชัวร์)
    await fs.promises.unlink(filePath);
    console.log(`[Cache] Deleted cache for map: ${mapName}`);
    return true;
  } catch (error) {
    // ถ้า Error code คือ 'ENOENT' แปลว่าไฟล์ไม่มีอยู่แล้ว (ถือว่าลบสำเร็จ)
    if (error.code === 'ENOENT') {
        return true; 
    }
    console.error(`[Cache] Failed to delete cache for map: ${mapName}`, error);
    return false;
  }
});

ipcMain.on('robot-command', (_, command) => {
  if (!rosWorker) {
    console.error('❌ Worker not initialized when sending robot-command');
    return;
  }
  rosWorker.postMessage({ type: 'sendCmd', command: command });
});

ipcMain.on('twist-command', (_, data) => {
  if (rosWorker) {
    rosWorker.postMessage({ type: 'sendTwist', data: data });
  }
});

ipcMain.on('ros:send-servo-tilt-int16', (_, angle) => {
  if (rosWorker) {
    rosWorker.postMessage({ type: 'sendServoTiltInt16',  angle });
  }
});

ipcMain.on('ros:send-servo-pan-int16', (_, angle) => {
  if (rosWorker) {
    rosWorker.postMessage({ type: 'sendServoPanInt16',  angle });
  }
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


ipcMain.handle('load:videos', async (event, customPath = null) => {
  const baseDir = customPath || path.join(app.getPath('videos'), 'ptR1');
  console.log('Loading videos asynchronously from:', baseDir);
  
  if (!fs.existsSync(baseDir)) return []; // เช็คแบบ Sync แค่ครั้งเดียวพอรับได้

  const allVideos = await getAllVideoFilesAsync(baseDir);
  allVideos.sort((a, b) => b.mtime - a.mtime); // เรียง ใหม่ -> เก่า
  return allVideos;
});

ipcMain.on('relay-command', (_, { relayId, command }) => {
  if (!rosWorker) {
    console.error('Worker not initialized when sending relay-command');
    return;
  }
  console.log(`Received relay command: ${relayId} → ${command}`);
  rosWorker.postMessage({
    type: 'sendRelay',
    relayId,
    command
  });
});

ipcMain.on('set-manual-mode', (event, { state }) => {
  if (state) {
    const command = 'manual_on';
    console.log(`Main: Switching MANUAL MODE ON → Send: ${command}`);
    rosWorker.postMessage({ type: 'sendCmd', command });
  } else {
    const command = 'manual_off';
    console.log(`Main: Switching MANUAL MODE OFF → Send: ${command}`);
    rosWorker.postMessage({ type: 'sendCmd', command });   
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
      // แนะนำ: ส่ง error กลับไปบอกหน้าเว็บ
      event.sender.send('video-save-status', { success: false, message: err.message });
      return;
    }
  
    // ตรวจสอบว่ามี FFmpeg ไหม (แบบง่าย)
    const cmd = `ffmpeg -y -i "${webmPath}" -c:v libx264 -c:a aac "${mp4Path}"`;
    exec(cmd, (error, stdout, stderr) => {
      if (error) {
        console.error(`FFmpeg error: ${error.message}`);
        console.warn("Make sure FFmpeg is installed and added to System PATH.");
        // แจ้งเตือนหน้าเว็บว่าแปลงไฟล์ล้มเหลว (แต่ไฟล์ webm ยังอยู่)
        event.sender.send('video-save-status', { success: false, warning: 'Saved WEBM but failed to convert to MP4. Check FFmpeg.' });
        return;
      }
      console.log(`Saved MP4: ${mp4Path}`);
      event.sender.send('video-save-status', { success: true, path: mp4Path });
    });
  });
});

app.on('before-quit', async () => {
  if (pythonProcess) {
    console.log('[Main] Killing Python backend...');
    pythonProcess.kill(); 
    pythonProcess = null;
  }
  if (rosWorker) {
     // ส่งคำสั่งหยุดสตรีมไปยัง Worker เพื่อให้เรียก Service /stream_manager/stop
     rosWorker.postMessage({ type: 'stopStream' });
     
     // ให้เวลา Worker ทำงานนิดนึงก่อน terminate
     await new Promise(r => setTimeout(r, 500));
     rosWorker.terminate();
  }
    if (rosWorker) {
    rosWorker.terminate();
  }
});

app.on('window-all-closed', function () {
  if (process.platform !== 'darwin') app.quit();
});

ipcMain.on('connect-rosbridge', (event, ip, port) => {
  const targetPort = port || '9090';
  const url = `ws://${ip}:${targetPort}`;
  rosWorker.postMessage({ type: 'connectROS', url: url });
  console.log(`Main: 🔌 Connecting to ROSBridge at ${url}`);
});

//เพิ่ม IPC handlers
ipcMain.handle('start-stream', async () => {
  return new Promise((resolve, reject) => {
    
    // 1. สร้างฟังก์ชันดักฟังคำตอบจาก Worker
    const responseHandler = (message) => {
      // เช็คว่าเป็นข้อความตอบกลับเรื่อง Stream ไหม
      if (message.type === 'startStreamResponse') {
        
        // ลบ Listener ออกเพื่อไม่ให้ Memory Leak
        rosWorker.off('message', responseHandler);
        
        if (message.success) {
          resolve(true); // บอก Frontend ว่าสำเร็จ
        } else {
          resolve(false); // บอก Frontend ว่าล้มเหลว
        }
      }
    };
    // 2. เริ่มดักฟัง
    rosWorker.on('message', responseHandler);
    // 3. ส่งคำสั่งไปหา Worker
    rosWorker.postMessage({ type: 'startStream' });
    //ตั้ง Timeout เผื่อ Worker เงียบหายไปเกิน 5 วิ
    setTimeout(() => {
        rosWorker.off('message', responseHandler);
        resolve(false); // หมดเวลา
    }, 7000);
  });
});
ipcMain.handle('stop-stream', async () => {
  return new Promise((resolve) => {
    const stopHandler = (message) => {
      if (message.type === 'stopStreamResponse') {
        rosWorker.off('message', stopHandler); // เลิกดักฟัง
        resolve(true); // บอก UI ว่าจบงานแล้ว
      }
    };
    rosWorker.on('message', stopHandler);
    rosWorker?.postMessage({ type: 'stopStream' });
    setTimeout(() => {
        rosWorker.off('message', stopHandler);
        resolve(true); 
    }, 3000);
  });
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

            const buffer = Buffer.from(msg.data.base64, 'base64');
            const pngFilePath = path.join(pngFolder, `${msg.data.name}.png`);
            fs.writeFileSync(pngFilePath, buffer);

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

    console.log(`[main]:  Synced maps: ${pendingMaps.join(', ')}`);
    mainWindow.webContents.send('sync-complete', imageArray);
  });
});

ipcMain.on('select-map', (event, mapName) => {
  rosWorker.postMessage({ type: 'loadMap', mapName });
});

ipcMain.on('save-map', (event, mapName) => {
  rosWorker.postMessage({ type: 'saveMap', mapName });
});

ipcMain.on('send-single-goal', (_, data) => {
  if (rosWorker) {
    rosWorker.postMessage({ type: 'sendSingleGoal', data });
  }
});

ipcMain.on('start-slam', () => {
  if (rosWorker) {
    rosWorker.postMessage({ type: 'startSLAM' });
  }
});

ipcMain.on('start-patrol', (_, { goals, loop }) => {
  if (rosWorker) rosWorker.postMessage({ type: 'startPatrol', goals, loop });
});

ipcMain.on('pause-patrol', () => {
  if (rosWorker) rosWorker.postMessage({ type: 'pausePatrol' });
});

ipcMain.on('resume-patrol', () => {
  if (rosWorker) rosWorker.postMessage({ type: 'resumePatrol' });
});

ipcMain.on('stop-patrol', () => {
  if (rosWorker) rosWorker.postMessage({ type: 'stopPatrol' });
});

ipcMain.on('nav:set-home', (_, mapName) => {
  if (rosWorker) rosWorker.postMessage({ type: 'setHome', mapName });
});

ipcMain.on('nav:go-home', (_, mapName) => {
  if (rosWorker) rosWorker.postMessage({ type: 'goHome', mapName });
});

ipcMain.on('nav:init-home', (_, mapName) => {
  if (rosWorker) rosWorker.postMessage({ type: 'initHome', mapName });
});

ipcMain.on('stop-slam', () => {
  if (rosWorker) rosWorker.postMessage({ type: 'stopSLAM' });
});

ipcMain.on('delete-map', (_, mapName) => {
  try {
        console.log(` Deleting map: ${mapName}`);

        //กำหนด Path ของโฟลเดอร์ maps ใน userData
        const localMapFolder = path.join(app.getPath('userData'), 'maps');
        const yamlFolder = path.join(localMapFolder, 'yaml');
        const pngFolder = path.join(localMapFolder, 'png');

        //กำหนดชื่อไฟล์ที่ต้องการลบ
        const filesToDelete = [
            path.join(yamlFolder, `${mapName}.yaml`), // ไฟล์ YAML
            path.join(pngFolder, `${mapName}.png`),   // ไฟล์ PNG
             // ถ้ามีไฟล์ .pgm ด้วย ก็ลบไปด้วยเลย
            path.join(yamlFolder, `${mapName}.pgm`)  
        ];

        //วนลูปเช็คและลบไฟล์
        filesToDelete.forEach(filePath => {
            if (fs.existsSync(filePath)) {
                fs.unlinkSync(filePath); // คำสั่งลบไฟล์
                console.log(`Deleted local file: ${filePath}`);
            } else {
                console.log(`File not found (skip): ${filePath}`);
            }
        });

        if (rosWorker) rosWorker.postMessage({ type: 'deleteMap', mapName });

        return { success: true, message: "Local files deleted" };

    } catch (error) {
        console.error("❌ Delete map error:", error);
        return { success: false, message: error.message };
    }

});

ipcMain.on('reset-slam', () => {
  if (rosWorker) rosWorker.postMessage({ type: 'resetSLAM' });
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

ipcMain.handle('save-edited-map', async (event, { newName, base64, yamlContent }) => {
    return new Promise((resolve, reject) => {
        if (!rosWorker) {
            resolve({ success: false, message: "Server/Worker not found" });
            return;
        }
        // 1. สร้าง Listener สำหรับดักฟัง Worker
        // ต้องสร้างเป็นตัวแปร function เพื่อให้เราสั่ง .off (ลบ listener) ได้เมื่อจบงาน
        const workerListener = (message) => {
            // เช็คว่าเป็นข้อความตอบกลับเรื่องนี้หรือเปล่า
            if (message.type === 'map-save-edited') {
                //ได้คำตอบแล้ว -> ลบ Listener ทิ้งทันที (Clean up)
                rosWorker.off('message', workerListener);
                // ส่ง data กลับไปที่ Frontend
                resolve(message.data);
            }
        };
        // 2. เริ่มดักฟัง (Listener) ที่ตัว Worker
        rosWorker.on('message', workerListener);
        // 3. ส่งคำสั่งไปที่ Worker
        rosWorker.postMessage({ 
            type: 'saveEditedMap',
            data: {
                name: newName, 
                base64: base64,
                yamlContent: yamlContent
            }
        });
        // 4. Timeout (เผื่อ Server ค้าง)
        setTimeout(() => {
            // หมดเวลา -> ลบ Listener ทิ้งเพื่อไม่ให้รก Memory
            rosWorker.off('message', workerListener);
            resolve({ success: false, message: "Timeout: ROS did not respond." });
        }, 10000);
    });
});
ipcMain.handle('get-userdata-path', (_, subfolder = '') => {
  return path.join(app.getPath('userData'), subfolder);
});

ipcMain.handle('dialog:select-folder-map', async (event, defaultPath = null) => {
  const result = await dialog.showOpenDialog({
    properties: ['openDirectory'] 
  });

  if (result.canceled) return null;
  const selectedDir = result.filePaths[0]; 
  console.log(`User selected folder: ${selectedDir}`);
  return selectedDir;
});

ipcMain.handle('get-env-is-dev', () => !app.isPackaged);

ipcMain.handle('get-video-path', (_, relativePath) => {
  const fullPath = path.join(app.getPath('videos'), 'ptR1', relativePath);
  // ใช้ protocol ที่เราสร้างเอง
  return `video://${fullPath.replace(/\\/g, '/')}`; 
});

function createWindow(ip) {
  const backendPath = path.join(__dirname, '../../python-backend/yolo_app.py');
  const pythonExecutable = path.join(__dirname, '../../python-backend/venv/bin/python');

  //pythonProcess = spawn(pythonExecutable, [backendPath], {stdio: 'inherit'});

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
  mainWindow.webContents.openDevTools();
  mainWindow.loadFile(path.join(__dirname, '../renderer/index.html'));
}
const { protocol } = require('electron');
app.whenReady().then(() => {
  protocol.registerFileProtocol('video', (request, callback) => {
    const url = request.url.replace('video://', '');
    try {
      return callback(decodeURI(url));
    } catch (error) {
      console.error(error);
    }
  });

  const mapFolder = path.join(app.getPath('userData'), 'maps');
  if (!fs.existsSync(mapFolder)) {
    fs.mkdirSync(mapFolder, { recursive: true });
    console.log('[main]:  Created userData/maps folder:', mapFolder);
  } else {
    console.log('[main]:  userData/maps already exists:', mapFolder);
  }
  startPythonBackend();
  createWindow();

  try {
    rosWorker = new Worker(path.join(__dirname, 'server.js'));
    rosWorker.on('message', (message) => {
      switch (message.type) {
        case 'map-data':
          break;
        case 'robot-pose-amcl':
          mainWindow?.webContents.send('robot-pose-amcl', message.data);
          break;
        case 'log':
          console.log('Worker Log:', message.data);
          break;
        case 'error':
          console.error('Worker Error:', message.data);
          break;
        case 'connection':
          const isConnected = message.data.isConnected;
            
          console.log(`ROS Connection Status: ${isConnected}`);
          mainWindow.webContents.send('connection-status', {
              connected: isConnected,
              message: isConnected ? 'Connected' : 'Disconnected'
          });
          break;
        case 'map-list':
          mainWindow.webContents.send('ros:map-list', message.data);
          break;
        case 'map-load':
          mainWindow.webContents.send('ros:map-load', message.data);
          break;
        case 'map-save':
          mainWindow.webContents.send('map-save-result', message.data);
          break;
        case 'map-base64':
          mainWindow.webContents.send('ros:map-base64', message.data);
          break;
        case 'map-save-result':
          mainWindow.webContents.send('map-save-result', message.data);
        case 'goal-result':
          console.log('[Main] Forwarding goal result to renderer:', message.data);   
          mainWindow?.webContents.send('goal-result', message.data);
          break;
        case 'slam-result':
          mainWindow?.webContents.send('slam-result', message.data);
          break;
        case 'slam-stop-result':
          mainWindow?.webContents.send('slam-stop-result', message.data);
          break;
        case 'slam-reset-result':
          mainWindow?.webContents.send('slam-reset-result', message.data);
          break;
        case 'live-map':
          mainWindow?.webContents.send('live-map', message.data);
          break;
        case 'robot-pose-slam':
          mainWindow?.webContents.send('robot-pose-slam', message.data);
          break;
        case 'planned-path':
          mainWindow?.webContents.send('planned-path', message.data);
          break;
        case 'stream-status': 
          mainWindow?.webContents.send('stream-status', message.data);
          break;
        case 'laser-scan-update':
          mainWindow?.webContents.send('laser-scan-data', message.data);
          break;
        case 'patrol-start-result':
            mainWindow?.webContents.send('patrol-start-result', message.data);
            break;
        case 'patrol-pause-result':
            mainWindow?.webContents.send('patrol-pause-result', message.data);
            break;
        case 'patrol-resume-result':
            mainWindow?.webContents.send('patrol-resume-result', message.data);
            break;
        case 'patrol-stop-result':
            mainWindow?.webContents.send('patrol-stop-result', message.data);
            break;
        case 'home-result':
            mainWindow?.webContents.send('nav:home-result', message.data);
            break;
        case 'robot-status-update':
            mainWindow?.webContents.send('robot-status', message.data);
            break;
        case 'patrol-status':
            mainWindow?.webContents.send('patrol-status-change', message.data);
            break;

        default:
          console.warn('[main]: Unknown message from worker:', message);
      }
    });
  
    rosWorker.on('error', (error) => {
      console.error('[main]: Worker Error:', error);
    });
  
    rosWorker.on('exit', (code) => {
      console.log(`[main]: Worker exited with code ${code}`);
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