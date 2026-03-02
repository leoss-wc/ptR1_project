// modules/inputControl.js

const ABSOLUTE_MAX_LINEAR = 1.1;
const ABSOLUTE_MAX_ANGULAR = 5.0;
const SERVO_STEP = 4;

let currentTiltAngle = 45;
let currentPanAngle = 90;
let speedMultiplier = 0.3;
const activeKeys = new Set();
let inputInterval = null;

export function initInputControl() {
  // Setup Slider
  const pwmSlider = document.getElementById('pwm-slider');
  const pwmValueLabel = document.getElementById('pwm-value-label');
  
  if (pwmSlider && pwmValueLabel) {
    pwmSlider.addEventListener('input', () => {
      const val = parseInt(pwmSlider.value);
      pwmValueLabel.textContent = val;
      speedMultiplier = val / 100;
    });
  }

  // Setup Key Listeners
  document.addEventListener('keydown', handleKeyDown);
  document.addEventListener('keyup', handleKeyUp);

  console.log('Input Control: Initialized keyboard and servo controls.');
}

function handleKeyDown(event) {
  if (event.repeat) return;
  activeKeys.add(event.code);
  console.log(`Key Down: ${event.code}`);

  if (!inputInterval && activeKeys.size > 0) {
    inputInterval = setInterval(processInputs, 100);
  }
}

function handleKeyUp(event) {
  activeKeys.delete(event.code);
  if (activeKeys.size === 0 && inputInterval) {
    clearInterval(inputInterval);
    inputInterval = null;
  }
}

function processInputs() {
  const modeLabel = document.getElementById('mode-label');

  let servoChanged = false;
  
  if (activeKeys.has('ArrowUp')) {
    currentTiltAngle = Math.max(20, currentTiltAngle - SERVO_STEP);
    servoChanged = true;
  }
  if (activeKeys.has('ArrowDown')) {
    currentTiltAngle = Math.min(105, currentTiltAngle + SERVO_STEP);
    servoChanged = true;
  }
  if (activeKeys.has('ArrowLeft')) {
    currentPanAngle = Math.min(110, currentPanAngle + SERVO_STEP);
    servoChanged = true;
  }
  if (activeKeys.has('ArrowRight')) {
    currentPanAngle = Math.max(75, currentPanAngle - SERVO_STEP);
    servoChanged = true;
  }

  if (servoChanged) {
    window.electronAPI.sendServoAngleTilt(currentTiltAngle);
    window.electronAPI.sendServoAnglePan(currentPanAngle);
    return;
  }

  if (modeLabel?.textContent.trim().toUpperCase() !== 'MANUAL ON') return;

  const currentMaxLinear = ABSOLUTE_MAX_LINEAR * speedMultiplier;
  const currentMaxAngular = ABSOLUTE_MAX_ANGULAR * speedMultiplier;

  let vx = 0, vy = 0, wz = 0;
  // แบบแก้ไข: ให้ A/D เป็นการเลี้ยว (Angular Z)
  if (activeKeys.has('KeyW')) vx += currentMaxLinear;
  if (activeKeys.has('KeyS')) vx -= currentMaxLinear;
  
  // ย้ายการเลี้ยวมาไว้ที่ A/D
  if (activeKeys.has('KeyA')) wz += currentMaxAngular; // เลี้ยวซ้าย
  if (activeKeys.has('KeyD')) wz -= currentMaxAngular; // เลี้ยวขวา

  // ถ้าหุ่นยนต์ของคุณเป็นล้อ Mecanum สามารถใช้ Q/E เพื่อสไลด์ข้างได้ (Optional)
  if (activeKeys.has('KeyQ')) vy += currentMaxLinear; 
  if (activeKeys.has('KeyE')) vy -= currentMaxLinear;

  window.electronAPI.sendTwistCommand({
    linear: { x: vx, y: vy, z: 0 },
    angular: { x: 0, y: 0, z: wz }
  });
  console.log(`Twist Command Sent: linear(${vx.toFixed(2)}, ${vy.toFixed(2)}, 0) angular(0, 0, ${wz.toFixed(2)})`);
}