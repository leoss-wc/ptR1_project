// robotState.js
// เก็บตำแหน่งและทิศทางของหุ่นยนต์ใน map (อัปเดตจาก ROS Topic)

export let robotTrail = []; // เก็บประวัติตำแหน่ง
const MAX_TRAIL_LENGTH = 100; // กำหนดความยาวสูงสุดของเส้นทาง

export const robotPose = {
  position: null,       // { x, y, z }
  orientation: null     // { x, y, z, w } → quaternion
};

// 🛠 ฟังก์ชันช่วยสำหรับอัปเดต robotPose
export function updateRobotPose(position, orientation) {
  robotPose.position = position;
  robotPose.orientation = orientation;

  //เพิ่มตำแหน่งใหม่เข้าไปใน Trail
  robotTrail.push({ ...position });

  //จำกัดความยาวของ Trail ไม่ให้ยาวเกินไป
  if (robotTrail.length > MAX_TRAIL_LENGTH) {
    robotTrail.shift(); // ลบจุดที่เก่าที่สุดออก
  }
}
