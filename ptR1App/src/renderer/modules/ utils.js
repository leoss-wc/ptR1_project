// modules/utils.js
// ฟังก์ชันช่วยเหลือต่างๆ ที่ใช้ในหลายโมดูล

// แปลงมุม Yaw เป็น Quaternion
export function yawToQuaternion(yaw) {
  const halfYaw = yaw / 2.0;
  return {
    x: 0,
    y: 0,
    z: Math.sin(halfYaw),
    w: Math.cos(halfYaw),
  };
}

// แปลง Quaternion เป็นมุม Yaw
export function getYawFromQuaternion(q) {
  if (!q) return 0;
  const { x, y, z, w } = q;
  return Math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}