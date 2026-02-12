export class OverlayCanvas {
    constructor(canvasId, videoElement) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.videoElement = videoElement;
        
        // 1. เรียก resize ครั้งแรก
        this.resize();

        // ✅ เพิ่ม: ResizeObserver เพื่อคอยดูว่าถ้าขนาดวิดีโอเปลี่ยน ให้แก้ขนาด Canvas ตามทันที
        this.observer = new ResizeObserver(() => this.resize());
        this.observer.observe(this.videoElement);
    }

    resize() {
        // ตั้งขนาด Canvas ให้เท่ากับขนาดที่แสดงผลจริงของ Video Element
        this.canvas.width = this.videoElement.clientWidth;
        this.canvas.height = this.videoElement.clientHeight;
    }

    clear() {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    }

    drawDetections(detections) {
        this.clear();
        
        // ป้องกันการหารด้วยศูนย์ ถ้าวิดีโอยังไม่โหลด
        if (!detections || detections.length === 0 || this.videoElement.videoWidth === 0) return;

        // คำนวณอัตราส่วน (Canvas Size / Video Resolution)
        const scaleX = this.canvas.width / this.videoElement.videoWidth;
        const scaleY = this.canvas.height / this.videoElement.videoHeight;

        detections.forEach(det => {
            const [x1, y1, x2, y2] = det.box;
            const label = `${det.class} ${det.confidence.toFixed(2)}`;
            
            // แปลงพิกัด
            const x = x1 * scaleX;
            const y = y1 * scaleY;
            const w = (x2 - x1) * scaleX;
            const h = (y2 - y1) * scaleY;

            // วาดกรอบ
            this.ctx.strokeStyle = '#00FF00'; // สีเขียวสว่าง
            this.ctx.lineWidth = 2;
            this.ctx.strokeRect(x, y, w, h);

            // ✅ เพิ่ม: วาดพื้นหลังตัวหนังสือ (ให้อ่านง่ายขึ้น)
            this.ctx.font = 'bold 14px sans-serif';
            const textWidth = this.ctx.measureText(label).width;
            const textHeight = 14; 
            const padding = 4;

            // ✅ เพิ่ม: เช็คว่าถ้าอยู่ชิดขอบบน ให้วาดตัวหนังสือไว้ข้างในกรอบแทน
            let textY = y - 5;
            if (y < 20) { 
                textY = y + 15; // ดันลงมาข้างล่างถ้าติดขอบบน
            }

            // วาดพื้นหลัง Label
            this.ctx.fillStyle = 'rgba(0, 255, 0, 0.7)'; // สีเขียวโปร่งแสง
            this.ctx.fillRect(x, textY - textHeight, textWidth + (padding * 2), textHeight + padding);

            // วาดตัวหนังสือ (สีดำตัดกับพื้นเขียว)
            this.ctx.fillStyle = 'black'; 
            this.ctx.fillText(label, x + padding, textY);
        });
    }

    // เพิ่มฟังก์ชันทำลาย Observer เมื่อไม่ใช้แล้ว (Optional)
    destroy() {
        if (this.observer) this.observer.disconnect();
    }
}