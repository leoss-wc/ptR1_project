// mapState.js
//เก็บข้อมูลแผนที่ที่กำลังใช้งาน (active map) และข้อมูลประกอบ เช่น base64 และ metadata

export const activeMap = {
  name: null,     // ชื่อแผนที่ที่เลือกอยู่
  base64: null,   // รูปแผนที่ในรูป base64
  meta: null      // ข้อมูล metadata เช่น resolution, origin (จาก YAML)
};
