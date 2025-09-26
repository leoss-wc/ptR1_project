const { contextBridge, ipcRenderer } = require('electron');

console.log('[PRELOAD] Using raw preload:', __filename);

contextBridge.exposeInMainWorld('robotControl', {
  sendKeyCommand: (command) => ipcRenderer.send('key-command', {command}),
  sendServoCommand: (command) => ipcRenderer.send('servo-command', {command}),
  sendCommand: (command) => ipcRenderer.send('uint32-command', {command}),
});

contextBridge.exposeInMainWorld('electronAPI', {
  loadVideosFromFolder: (pathOverride) => ipcRenderer.invoke('load:videos', pathOverride),
  getVideoFileURL: (relativePath) => ipcRenderer.invoke('get-video-path', relativePath),
  startFFmpegStream: () => ipcRenderer.send('start-stream'),
  stopFFmpegStream: () => ipcRenderer.send('stop-stream'),
  onStreamStatus: (callback) => ipcRenderer.on('stream-status', (_, data) => callback(data)),

  loadRobots: () => ipcRenderer.invoke('robots:load'),
  saveRobots: (robots) => ipcRenderer.invoke('robots:save', robots),


  //ROSBridge related api
  connectROSBridge: (ip) => ipcRenderer.send('connect-rosbridge', ip),

  sendRelayCommand: (relayId, command) => ipcRenderer.send('relay-command', { relayId, command }),
  setManualMode: (state) => ipcRenderer.send('set-manual-mode', { state }),
  
  loadVideosFromFolder: (customPath) => ipcRenderer.invoke('load:videos', customPath),
  saveVideo: ({ buffer, date, filename }) => {
      const nodeBuffer = Buffer.from(buffer);
      ipcRenderer.send('save-video', { buffer: nodeBuffer, date, filename });
    },
  onImage: (callback) => ipcRenderer.on('camera:image', (_, data) => callback(data)),
  onPowerUpdate: (callback) => ipcRenderer.on('power', (_, data) => callback(data)),
  sendCommand_vairable: (variableId, value) => {ipcRenderer.send('uint32-command', { variableId, value });},
  onConnectionStatus: (callback) => {ipcRenderer.on('connection-status', (_, status) => callback(status));},
    
    // Map related api for patrol functions
  onSyncComplete: (callback) => ipcRenderer.on('sync-complete', (_, maps) => callback(maps)),
  syncMaps: () => ipcRenderer.send('sync-maps'),
  getLocalMaps: () => ipcRenderer.invoke('get-local-maps'),
  selectFolder_video: () => ipcRenderer.invoke('dialog:select-folder'),
  getDefaultVideoPath: () => ipcRenderer.invoke('get-default-video-path'),
  selectFolder: (defaultPath = null) => ipcRenderer.invoke('dialog:select-folder-map', defaultPath),
  getUserDataPath: (subfolder = '') => ipcRenderer.invoke('get-userdata-path', subfolder),
  selectMap: (mapName) => ipcRenderer.send('select-map', mapName),
  getMapDataByName: (name) => ipcRenderer.invoke('get-map-data-by-name', name),
  saveMapCache: (mapName, imageData) => ipcRenderer.invoke('mapcache:save', { mapName, imageData }),
  loadMapCache: (mapName) => ipcRenderer.invoke('mapcache:load', mapName),

  // Robot pose and planned path api
  onRobotPose: (callback) => ipcRenderer.on('robot-pose', (event, ...args) => callback(...args)),
  onPlannedPath: (callback) => ipcRenderer.on('planned-path', (event, ...args) => callback(...args)),
  setInitialPose: (pose) => ipcRenderer.send('set-initial-pose', pose),

  // Patrol related api
  sendPatrolPath: (pathArray) => ipcRenderer.send('send-patrol-path', pathArray),
  sendStopPatrol: () => ipcRenderer.send('send-stop-patrol'),
  sendSingleGoal: (pt) => ipcRenderer.send('send-single-goal', pt),
  resumePatrol: (path, index) => ipcRenderer.send('resume-patrol', { path, index }),
  cancelCurrentGoal: () => ipcRenderer.send('send-stop-patrol'),
  onGoalResult: (callback) => ipcRenderer.on('goal-result', (_, data) => callback(data)),


  // SLAM related api
  saveMap: (mapName) => ipcRenderer.send('save-map', mapName),
  onMapSaveResult: (callback) => ipcRenderer.on('map-save-result', (_, result) => callback(result)),
  startSLAM: () => ipcRenderer.send('start-slam'),
  stopSLAM: () => ipcRenderer.send('stop-slam'),
  onSLAMStartResult: (cb) => ipcRenderer.on('slam-start-result', (_, data) => cb(data)),
  onSLAMStopResult: (cb) => ipcRenderer.on('slam-stop-result', (_, data) => cb(data)),
  onLiveMap: (cb) => ipcRenderer.on('live-map', (_, data) => cb(data)),

  //Home map canvas api
  onRobotPose: (cb) => ipcRenderer.on('robot-pose', (_, data) => cb(data)),
  getMapMeta: (mapName) => ipcRenderer.invoke('get-map-meta', mapName),

  // Settings related api
  saveSettings: (settings) => ipcRenderer.invoke('settings:save', settings),
  loadSettings: () => ipcRenderer.invoke('settings:load'),

});




