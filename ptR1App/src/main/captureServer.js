/**
 * captureServer.js — ROS Worker (server.js) Additions
 * จัดการการสื่อสารกับ ROS Topic/Service บน Raspi
 *
 * วิธีใช้: เพิ่มใน server.js
 *
 *  1. import ที่หัวไฟล์:
 *     const { handleCaptureMessage, initCaptureSubscriber } = require('./captureServer');
 *
 *  2. ใน switch-case ของ parentPort.on('message') เพิ่ม:
 *     case 'captureSnapshot':
 *     case 'captureBurstStart':
 *     case 'captureBurstStop':
 *       handleCaptureMessage(message, ros, parentPort);
 *       break;
 *
 *  3. ใน connectROSBridge() หลัง subscribe อื่นๆ เพิ่ม:
 *     initCaptureSubscriber(ros, parentPort);
 */

const ROSLIB = require('roslib'); // server.js โหลดอยู่แล้ว แต่ใส่ไว้เพื่อความชัดเจน

// ---- 1. Handle messages จาก Main Process --------------------------------

/**
 * @param {object}         message    — message จาก parentPort
 * @param {ROSLIB.Ros}     ros        — ROS connection instance
 * @param {MessagePort}    parentPort — Worker port ส่งกลับ Main
 */
function handleCaptureMessage(message, ros, parentPort) {
  if (!ros || !ros.isConnected) {
    console.warn('[captureServer] ROS not connected, ignoring:', message.type);
    return;
  }

  switch (message.type) {

    // ---- ถ่ายภาพครั้งเดียว -----------------------------------------------
    case 'captureSnapshot': {
      // ส่ง label ผ่าน topic ก่อน แล้วเรียก Service
      _callCaptureService(ros, parentPort, '/stream_manager/capture');
      break;
    }

    // ---- เริ่ม Burst -------------------------------------------------------
    case 'captureBurstStart': {
      const label    = message.label    || 'object';
      const interval = message.interval || 2.0;
      // รูปแบบ: "label:interval_sec"  เช่น "fire:2.0"
      _publishCaptureConfig(ros, `${label}:${interval}`);
      console.log(`[captureServer] Burst started: label=${label}, interval=${interval}s`);
      break;
    }

    // ---- หยุด Burst --------------------------------------------------------
    case 'captureBurstStop': {
      _publishCaptureConfig(ros, 'stop');
      _callCaptureService(ros, parentPort, '/stream_manager/capture_stop');
      console.log('[captureServer] Burst stopped');
      break;
    }
  }
}

// ---- 2. Subscribe รับผลจาก Raspi -----------------------------------------

/**
 * Subscribe topic /stream_manager/captured
 * Raspi publish JSON: { filename, count, label }
 * แล้ว relay ไปให้ Main Process
 */
function initCaptureSubscriber(ros, parentPort) {
  if (!ros || !ros.isConnected) return;

  const capturedTopic = new ROSLIB.Topic({
    ros,
    name:        '/stream_manager/captured',
    messageType: 'std_msgs/String'
  });

  capturedTopic.subscribe((msg) => {
    try {
      const data = JSON.parse(msg.data);
      // ส่งกลับไป Main Process เพื่อ relay ไป renderer
      parentPort.postMessage({ type: 'capture-result', data });
      console.log(`[captureServer] Captured: ${data.filename} (total: ${data.count})`);
    } catch (e) {
      console.error('[captureServer] Failed to parse captured msg:', e);
    }
  });

  console.log('[captureServer] Subscribed to /stream_manager/captured');
}

// ---- Private Helpers -------------------------------------------------------

/** Publish config string ไป /stream_manager/capture_config */
function _publishCaptureConfig(ros, payload) {
  const configTopic = new ROSLIB.Topic({
    ros,
    name:        '/stream_manager/capture_config',
    messageType: 'std_msgs/String'
  });
  configTopic.publish(new ROSLIB.Message({ data: payload }));
}

/** Call Trigger service (สำหรับ capture หรือ capture_stop) */
function _callCaptureService(ros, parentPort, serviceName) {
  const service = new ROSLIB.Service({
    ros,
    name:        serviceName,
    serviceType: 'std_srvs/Trigger'
  });
  service.callService(new ROSLIB.ServiceRequest({}), (result) => {
    console.log(`[captureServer] ${serviceName}:`, result.message);
  }, (err) => {
    console.error(`[captureServer] ${serviceName} Error:`, err);
  });
}

module.exports = { handleCaptureMessage, initCaptureSubscriber };
