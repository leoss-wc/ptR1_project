// modules/FrameProcessor.js
export class FrameProcessor {
    constructor(videoElement, onMessageCallback, wsUrl = 'ws://localhost:8765') {
        this.videoElement = videoElement;
        this.onMessageCallback = onMessageCallback; // Callback เพื่อส่งผลลัพธ์กลับ
        this.wsUrl = wsUrl;
        this.ws = null;
        
        // สร้าง Canvas รอไว้เลย
        this.canvas = document.createElement('canvas');
        // willReadFrequently ช่วยเพิ่มประสิทธิภาพเมื่อใช้ drawImage/toDataURL บ่อยๆ
        this.ctx = this.canvas.getContext('2d', { willReadFrequently: true }); 
        
        this.isProcessing = false; // ตัวแปรคุมสถานะการทำงาน
    }

    start() {
        if (this.isProcessing) return; // ป้องกันการ start ซ้ำ
        this.isProcessing = true;
        
        console.log(`Attempting to connect to YOLO backend at ${this.wsUrl}`);
        
        // สร้าง connection ใหม่
        this.ws = new WebSocket(this.wsUrl);

        this.ws.onopen = () => {
            console.log("WebSocket connection to YOLO backend established.");
            // ✅ แก้ไข: เรียกชื่อฟังก์ชันให้ถูกต้อง (จาก sendFrameLoop เป็น sendFrame)
            this.sendFrame(); 
        };

        this.ws.onmessage = (event) => {
            // ถ้าสั่งหยุดแล้ว ไม่ต้องทำต่อ
            if (!this.isProcessing) return;

            // 1. จัดการผลลัพธ์จาก Python
            try {
                const data = JSON.parse(event.data);
                if (this.onMessageCallback) {
                    this.onMessageCallback(data);
                }
            } catch (e) {
                console.error("Error parsing YOLO result:", e);
            }

            // 2. ส่งภาพถัดไป (Loop แบบ Ping-Pong)
            // รอ Python ตอบกลับมาก่อนค่อยส่งภาพใหม่ เพื่อไม่ให้ Server รับภาระหนักเกินไป
            requestAnimationFrame(() => this.sendFrame());
        };

        this.ws.onclose = () => {
            console.log("WebSocket connection to YOLO backend closed.");
            this.isProcessing = false;
            this.ws = null;
        };

        this.ws.onerror = (error) => {
            console.error("WebSocket YOLO error:", error);
            this.stop();
        };
    }

    stop() {
        this.isProcessing = false; // ตัด Loop ทันที
        if (this.ws) {
            this.ws.close();
            this.ws = null;
            console.log("Frame processor stopped.");
        }
    }

    sendFrame() {
        // เช็คเงื่อนไขความปลอดภัย
        if (!this.isProcessing || !this.ws || this.ws.readyState !== WebSocket.OPEN) return;

        // เช็คว่าวิดีโอพร้อมหรือยัง
        if (this.videoElement.videoWidth === 0 || this.videoElement.videoHeight === 0) {
            // ถ้าวิดีโอยังไม่มา ให้รอรอบหน้า
            if (this.isProcessing) {
                requestAnimationFrame(() => this.sendFrame());
            }
            return;
        }

        // ปรับขนาด Canvas ให้เท่ากับวิดีโอ (ทำเมื่อขนาดเปลี่ยน)
        if (this.canvas.width !== this.videoElement.videoWidth || this.canvas.height !== this.videoElement.videoHeight) {
            this.canvas.width = this.videoElement.videoWidth;
            this.canvas.height = this.videoElement.videoHeight;
        }

        // วาดภาพลง Canvas
        this.ctx.drawImage(this.videoElement, 0, 0, this.canvas.width, this.canvas.height);

        // ส่งข้อมูลแบบ Base64 (JPEG Quality 0.5 กำลังดีสำหรับ Realtime)
        const dataURL = this.canvas.toDataURL('image/jpeg', 0.5);
        
        if (this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(dataURL);
        }
    }
}