// modules/FrameProcessor.js
export class FrameProcessor {
    constructor(videoElement, onMessageCallback, wsUrl = 'ws://localhost:8765') {
        this.videoElement = videoElement;
        this.onMessageCallback = onMessageCallback; // Callback เพื่อส่งผลลัพธ์กลับ
        this.wsUrl = wsUrl;
        this.ws = null;
        this.canvas = document.createElement('canvas');
        this.ctx = this.canvas.getContext('2d');
        this.animationFrameId = null;
        this.isProcessing = false; // ตัวแปรกันส่งซ้อน
    }

    start() {
        if (this.ws) return; // ป้องกันการ connect ซ้ำ
        
        console.log(`Attempting to connect to YOLO backend at ${this.wsUrl}`);
        this.ws = new WebSocket(this.wsUrl);

        this.ws.onopen = () => {
            console.log("WebSocket connection to YOLO backend established.");
            this.sendFrameLoop(); // เริ่มส่งภาพแรกเพื่อกระตุ้นลูป
        };

        this.ws.onmessage = (event) => {
            //ได้รับผลลัพธ์จาก Python
            try {
                const data = JSON.parse(event.data);
                if (this.onMessageCallback) {
                    this.onMessageCallback(data);
                }
            } catch (e) {
                console.error("Error parsing YOLO result:", e);
            }

            // เมื่อได้รับผลแล้ว ค่อยส่งภาพถัดไป (Ping-Pong)
            // ใช้ requestAnimationFrame เพื่อไม่ให้หนัก Browser เกินไป
            requestAnimationFrame(() => this.sendFrame());
        };

        this.ws.onclose = () => {
            console.log("WebSocket connection to YOLO backend closed.");
            this.ws = null;
        };

        this.ws.onerror = (error) => {
            console.error("WebSocket YOLO error:", error);
            this.stop();
        };
    }

    stop() {
        if (this.ws) {
            this.ws.close();
            this.ws = null;
            console.log("Frame processor stopped.");
        }
    }

    sendFrame() {
        // เช็คสถานะ Connection
        if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;

        // เช็คว่าวิดีโอพร้อมหรือยัง (แก้ปัญหา Canvas width=0)
        if (this.videoElement.videoWidth === 0 || this.videoElement.videoHeight === 0) {
            // ถ้าวิดีโอยังไม่มา ให้รอก่อนแล้วลองใหม่
            requestAnimationFrame(() => this.sendFrame());
            return;
        }

        // ปรับขนาด Canvas ให้เท่ากับวิดีโอ (ทำครั้งเดียวหรือทำทุกครั้งก็ได้ถ้าขนาดเปลี่ยน)
        if (this.canvas.width !== this.videoElement.videoWidth) {
            this.canvas.width = this.videoElement.videoWidth;
            this.canvas.height = this.videoElement.videoHeight;
        }

        // วาดภาพลง Canvas
        this.ctx.drawImage(this.videoElement, 0, 0, this.canvas.width, this.canvas.height);

        // ส่งข้อมูล (ลด Quality ลงเหลือ 0.5-0.7 เพื่อความเร็ว)
        const dataURL = this.canvas.toDataURL('image/jpeg', 0.6);
        this.ws.send(dataURL);
    }
}