// modules/inputControl.js

const ABSOLUTE_MAX_LINEAR = 1.0;
const ABSOLUTE_MAX_ANGULAR = 2.0;
const SERVO_STEP = 2;

let currentTiltAngle = 90;
let currentPanAngle = 90;
let speedMultiplier = 0.7;
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
}

function handleKeyDown(event) {
  if (event.repeat) return;
  activeKeys.add(event.code);

  // Servo Control Logic
  let servoChanged = false;
  if (activeKeys.has('ArrowUp')) {
    currentTiltAngle = Math.min(180, currentTiltAngle + SERVO_STEP);
    servoChanged = true;
  }
  if (activeKeys.has('ArrowDown')) {
    currentTiltAngle = Math.max(0, currentTiltAngle - SERVO_STEP);
    servoChanged = true;
  }
  if (activeKeys.has('ArrowLeft')) {
    currentPanAngle = Math.min(180, currentPanAngle + SERVO_STEP);
    servoChanged = true;
  }
  if (activeKeys.has('ArrowRight')) {
    currentPanAngle = Math.max(0, currentPanAngle - SERVO_STEP);
    servoChanged = true;
  }

  if (servoChanged) {
    window.electronAPI.sendServoAngleTilt(currentTiltAngle);
    window.electronAPI.sendServoAnglePan(currentPanAngle);
  }

  if (!inputInterval && activeKeys.size > 0) {
    inputInterval = setInterval(processInputs, 100);
  }
}

function handleKeyUp(event) {
  activeKeys.delete(event.code);
  if (activeKeys.size === 0 && inputInterval) {
    clearInterval(inputInterval);
    inputInterval = null;
    window.electronAPI.sendTwistCommand({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
  }
}

function processInputs() {
  const modeLabel = document.getElementById('mode-label');
  if (modeLabel?.textContent.trim().toUpperCase() !== 'MANUAL ON') return;

  const currentMaxLinear = ABSOLUTE_MAX_LINEAR * speedMultiplier;
  const currentMaxAngular = ABSOLUTE_MAX_ANGULAR * speedMultiplier;

  let vx = 0, vy = 0, wz = 0;
  if (activeKeys.has('KeyW')) vx += currentMaxLinear;
  if (activeKeys.has('KeyS')) vx -= currentMaxLinear;
  if (activeKeys.has('KeyA')) vy += currentMaxLinear;
  if (activeKeys.has('KeyD')) vy -= currentMaxLinear;
  if (activeKeys.has('KeyQ')) wz += currentMaxAngular;
  if (activeKeys.has('KeyE')) wz -= currentMaxAngular;

  window.electronAPI.sendTwistCommand({
    linear: { x: vx, y: vy, z: 0 },
    angular: { x: 0, y: 0, z: wz }
  });
}