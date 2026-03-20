const { contextBridge, ipcRenderer } = require('electron');

console.log('[PRELOAD] Using raw preload:', __filename);

contextBridge.exposeInMainWorld('electronAPI', {
  getVideoFileURL: (relativePath) => ipcRenderer.invoke('get-video-path', relativePath),
  onVideoSaveStatus: (callback) => ipcRenderer.on('video-save-status', (_, result) => callback(result)),
  startFFmpegStream: () => ipcRenderer.invoke('start-stream'),
  stopFFmpegStream: () => ipcRenderer.invoke('stop-stream'),
  onStreamStatus: (callback) => ipcRenderer.on('stream-status', (_, data) => callback(data)),
  saveDatasetImage: (base64Data) => ipcRenderer.send('save-dataset-image', base64Data),

  loadRobots: () => ipcRenderer.invoke('robots:load'),
  saveRobots: (robots) => ipcRenderer.invoke('robots:save', robots),
  onRobotStatus: (callback) => {
    ipcRenderer.on('robot-status', (_event, data) => {
    callback(data);
    });
  },
  onSystemPi: (callback) => ipcRenderer.on('system-profile-update', (event, data) => callback(data)),
  // Robot movement related api
  sendTwistCommand: (data) => ipcRenderer.send('twist-command', data),
  sendServoAngleTilt: (angle) => ipcRenderer.send('ros:send-servo-tilt-int16', angle),
  sendServoAnglePan: (angle) => ipcRenderer.send('ros:send-servo-pan-int16', angle),
  sendCommand: (command) => ipcRenderer.send('robot-command', command),

  //ROSBridge related api
  connectROSBridge: (ip) => ipcRenderer.send('connect-rosbridge', ip),

  sendRelayCommand: (command) => ipcRenderer.send('relay-command', { command }),
  setManualMode: (state) => ipcRenderer.send('set-manual-mode', { state }),
  
  loadVideosFromFolder: (customPath) => ipcRenderer.invoke('load:videos', customPath),
  saveVideo: ({ buffer, date, filename }) => {
    ipcRenderer.invoke('save-video', { buffer, date, filename }); // ← invoke แทน send
},
  onImage: (callback) => ipcRenderer.on('camera:image', (_, data) => callback(data)),
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
  selectMap: (name) => ipcRenderer.invoke('map-select', name),
  getMapDataByName: (name) => ipcRenderer.invoke('get-map-data-by-name', name),
  saveMapCache: (mapName, imageData) => ipcRenderer.invoke('mapcache:save', { mapName, imageData }),
  loadMapCache: (mapName) => ipcRenderer.invoke('mapcache:load', mapName),
  deleteMapCache: (mapName) => ipcRenderer.invoke('mapcache:delete', mapName),
  saveEditedMap: (newName, base64, yamlContent) => 
        ipcRenderer.invoke('save-edited-map', { newName, base64, yamlContent }),
  onLaserScan: (callback) => ipcRenderer.on('laser-scan-data', (_event, value) => callback(value)),
  deleteMap: (mapName) => ipcRenderer.send('delete-map', mapName),
  onMapDeleteResult: (callback) => ipcRenderer.on('map-delete-result', (_, result) => callback(result)),
  getMapHome: (mapName) => ipcRenderer.invoke('nav:get-home', mapName),

  // Robot pose and planned path api
  onRobotPosSlam: (callback) => ipcRenderer.on('robot-pose-slam', (event, ...args) => callback(...args)),
  onRobotPosAmcl: (callback) => ipcRenderer.on('robot-pose-amcl', (event, ...args) => callback(...args)),
  resetSLAM: () => ipcRenderer.send('reset-slam'),
  onSLAMResetResult: (callback) => ipcRenderer.on('slam-reset-result', (_, result) => callback(result)),
  onTfUpdate: (callback) => ipcRenderer.on('tf-update', (_event, value) => callback(value)),
  stopNavigation: (savePose) => ipcRenderer.invoke('nav-stop', savePose),
  startNavigation: (restorePose) => ipcRenderer.invoke('nav-start', restorePose),


  switchPoseSubscriber: (mode) => ipcRenderer.send('switch-pose-subscriber', { mode }),
  onPlannedPath: (callback) => ipcRenderer.on('planned-path', (event, ...args) => callback(...args)),
  setInitialPose: (pose) => ipcRenderer.send('set-initial-pose', pose),
  setHome: (mapName) => ipcRenderer.invoke('nav:set-home', mapName),
  goHome: (mapName) => ipcRenderer.invoke('nav:go-home', mapName),
  initHome: (mapName) => ipcRenderer.invoke('nav-init-home', mapName),

  onHomeResult: (callback) => ipcRenderer.on('nav:home-result', (_, result) => callback(result)),

  // Patrol related api
  cancelCurrentGoal: () => ipcRenderer.send('send-stop-patrol'),
  onGoalResult: (callback) => ipcRenderer.on('goal-result', (_, data) => callback(data)),
  onGoalResultOnce: (callback) => ipcRenderer.once('goal-result', (_, data) => callback(data)),

  startPatrol: (goals, loop) => ipcRenderer.send('start-patrol', { goals, loop }),
  onPatrolStartResult: (callback) => ipcRenderer.on('patrol-start-result', (_, result) => callback(result)),

  pausePatrol: () => ipcRenderer.send('pause-patrol'),
  onPatrolPauseResult: (callback) => ipcRenderer.on('patrol-pause-result', (_, result) => callback(result)),

  resumePatrol: () => ipcRenderer.send('resume-patrol'),
  onPatrolResumeResult: (callback) => ipcRenderer.on('patrol-resume-result', (_, result) => callback(result)),
  
  stopPatrol: () => ipcRenderer.send('stop-patrol'),
  onPatrolStopResult: (callback) => ipcRenderer.on('patrol-stop-result', (_, result) => callback(result)),
  onPatrolStatusChange: (callback) => ipcRenderer.on('patrol-status', (_, status) => callback(status)),


  // SLAM related api
  saveMap: (mapName) => ipcRenderer.send('save-map', mapName),
  onMapSaveResult: (callback) => ipcRenderer.on('map-save-result', (_, result) => callback(result)),
  startSLAM: () => ipcRenderer.send('start-slam'),
  stopSLAM: () => ipcRenderer.send('stop-slam'),
  onSLAMStartResult: (cb) => ipcRenderer.on('slam-result', (_, data) => cb(data)),
  onSLAMStopResult: (cb) => ipcRenderer.on('slam-stop-result', (_, data) => cb(data)),
  onLiveMap: (callback) => ipcRenderer.on('live-map', (_event, value) => callback(value)),

  //Home map canvas api
  getMapMeta: (mapName) => ipcRenderer.invoke('get-map-meta', mapName),

  // Settings related api
  saveSettings: (settings) => ipcRenderer.invoke('settings:save', settings),
  loadSettings: () => ipcRenderer.invoke('settings:load'),

  // Detection settings related api
  updateDetection: (settings) => ipcRenderer.invoke('detection:update', settings),
  onDetectionStatus: (callback) => ipcRenderer.on('detection-status', (_, data) => callback(data)),
  onDetectionAlert: (callback) => ipcRenderer.on('detection-alert', (_, data) => callback(data)),
  captureSnapshot:    (label)            => ipcRenderer.invoke('capture:single', label),
  captureStartBurst:  ({ label, interval }) => ipcRenderer.invoke('capture:startBurst', { label, interval }),
  captureStopBurst:   ()                 => ipcRenderer.invoke('capture:stopBurst'),
  captureOpenFolder:  (label)            => ipcRenderer.invoke('capture:openFolder', label),
  captureGetStats:    ()                 => ipcRenderer.invoke('capture:getStats'),
  onCaptureResult:    (callback)         => ipcRenderer.on('capture:result', (_, data) => callback(data)),


});




