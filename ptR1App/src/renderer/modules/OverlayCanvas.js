export class OverlayCanvas {
    constructor(canvasId, videoElement) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.videoElement = videoElement;

        // Default Config
        this.securityConfig = {
            danger: ['fire', 'knife', 'weapon'],
            restricted: ['person'], 
        };

        this.restrictedTime = { start: "22:00", end: "06:00", enabled: false };
        this.securityModeEnabled = true;

        
        //เรียก resize ครั้งแรก
        this.resize();
        //ResizeObserver เพื่อคอยดูว่าถ้าขนาดวิดีโอเปลี่ยน ให้แก้ขนาด Canvas ตามทันที
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
    setSecurityMode(enabled) {
        this.securityModeEnabled = enabled;
    }

    setRestrictedTime(start, end, enabled = true) {
        this.restrictedTime = { start, end, enabled };
    }
    isCurrentTimeRestricted() {
        // ถ้าไม่ได้เปิดใช้งาน Schedule (enabled = false) 
        // ให้ถือว่าแจ้งเตือนตลอดเวลา (return true)
        if (!this.restrictedTime.enabled) return true;

        const now = new Date();
        // getHours() คืนค่า 0-23 (24-hour format)
        const currentMinutes = now.getHours() * 60 + now.getMinutes();

        // แปลงเวลา Start/End ที่รับมา ("22:00", "06:00") เป็นนาทีรวม
        const [startH, startM] = this.restrictedTime.start.split(':').map(Number);
        const [endH, endM] = this.restrictedTime.end.split(':').map(Number);
        
        const startTotal = startH * 60 + startM;
        const endTotal = endH * 60 + endM;

        // กรณีช่วงเวลาปกติ (เช่น 08:00 - 17:00)
        if (startTotal < endTotal) {
            return currentMinutes >= startTotal && currentMinutes <= endTotal;
        } 
        // กรณีข้ามคืน (เช่น 22:00 - 06:00)
        // ต้องตรวจสอบว่าเวลาปัจจุบัน "มากกว่าเวลาเริ่ม" หรือ "น้อยกว่าเวลาจบ"
        else {
            return currentMinutes >= startTotal || currentMinutes <= endTotal;
        }
    }
    setRestrictedItems(items) {
        this.securityConfig.restricted = items;
        console.log("Restricted Items Updated:", this.securityConfig.restricted);
    }

    drawDetections(detections) {
        this.clear();
        
        // ป้องกันการหารด้วยศูนย์ ถ้าวิดีโอยังไม่โหลด
        if (!detections || detections.length === 0 || this.videoElement.videoWidth === 0) return;

        // คำนวณอัตราส่วน (Canvas Size / Video Resolution)
        const scaleX = this.canvas.width / this.videoElement.videoWidth;
        const scaleY = this.canvas.height / this.videoElement.videoHeight;
        // isFlashOn สำหรับเอฟเฟกต์กระพริบ (เปลี่ยนค่าทุก 500ms)
        const isFlashOn = Math.floor(Date.now() / 500) % 2 === 0;

        detections.forEach(det => {
            const [x1, y1, x2, y2] = det.box;
            let alertLevel = 'normal';
            
            if (this.securityModeEnabled) {
                // 1. เช็คของอันตรายร้ายแรง (Danger)
                if (this.securityConfig.danger.includes(det.class)) {
                    alertLevel = 'danger';
                } 
                // 2. เช็คของใน Check List (Restricted)
                else if (this.securityConfig.restricted.includes(det.class)) {
                    // ถ้าอยู่ในรายการที่ติ๊กไว้ -> เช็คเวลาต่อ
                    if (this.isCurrentTimeRestricted()) {
                        alertLevel = 'restricted';
                    }
                }
            }

            let color = '#00FF00'; 
            let lineWidth = 2;
            let labelPrefix = '';
            let label = `${det.class} ${Math.round(det.confidence * 100)}%`;

            if (alertLevel === 'danger') {
                color = isFlashOn ? '#FF0000' : '#FFFF00'; 
                lineWidth = 5;
                labelPrefix = 'DANGER: ';
            } else if (alertLevel === 'restricted') {
                color = isFlashOn ? '#FF4500' : '#FF0000'; 
                lineWidth = 4;
                labelPrefix = 'ALERT: ';
            }
            label = labelPrefix + label;

            const x = x1 * scaleX;
            const y = y1 * scaleY;
            const w = (x2 - x1) * scaleX;
            const h = (y2 - y1) * scaleY;

            this.ctx.beginPath();
            this.ctx.strokeStyle = color;
            this.ctx.lineWidth = lineWidth;
            this.ctx.strokeRect(x, y, w, h);

            if (alertLevel !== 'normal') {
                this.ctx.strokeStyle = color;
                this.ctx.globalAlpha = 0.3;
                this.ctx.lineWidth = lineWidth + 4;
                this.ctx.strokeRect(x, y, w, h);
                this.ctx.globalAlpha = 1.0; 
            }

            this.ctx.font = alertLevel === 'normal' ? 'bold 14px sans-serif' : 'bold 16px sans-serif';
            const textHeight = alertLevel === 'normal' ? 14 : 20; 
            const padding = 6;
            let textY = y - 5;
            if (y < 30) textY = y + 25;

            this.ctx.fillStyle = color; 
            this.ctx.globalAlpha = alertLevel === 'normal' ? 0.7 : 1.0; 
            const textWidth = this.ctx.measureText(label).width;
            this.ctx.fillRect(x, textY - textHeight, textWidth + (padding * 2), textHeight + padding);
            this.ctx.globalAlpha = 1.0;

            this.ctx.fillStyle = (alertLevel === 'danger' || alertLevel === 'restricted') ? 'white' : 'black';
            this.ctx.fillText(label, x + padding, textY);
        });

    }

    // เพิ่มฟังก์ชันทำลาย Observer เมื่อไม่ใช้แล้ว (Optional)
    destroy() {
        if (this.observer) this.observer.disconnect();
    }
}