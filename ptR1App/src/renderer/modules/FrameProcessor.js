export class FrameProcessor {
    constructor(videoElement, onMessageCallback, wsUrl = 'ws://localhost:8765') {
        this.videoElement = videoElement;
        this.onMessageCallback = onMessageCallback; // Callback เพื่อส่งผลลัพธ์กลับ
        this.wsUrl = wsUrl;
        this.ws = null;
        this.canvas = document.createElement('canvas');
        this.ctx = this.canvas.getContext('2d');
        this.animationFrameId = null;
    }

    start() {
        if (this.animationFrameId) return;
        console.log(`Attempting to connect to YOLO backend at ${this.wsUrl}`);
        this.ws = new WebSocket(this.wsUrl);

        this.ws.onopen = () => {
            console.log("WebSocket connection to YOLO backend established.");
            this.canvas.width = this.videoElement.videoWidth;
            this.canvas.height = this.videoElement.videoHeight;
            this.sendFrameLoop();
        };

        this.ws.onmessage = (event) => {
            // เมื่อได้รับผลลัพธ์จาก Python ให้เรียก Callback กลับไป
            if (this.onMessageCallback) {
                this.onMessageCallback(JSON.parse(event.data));
            }
        };

        this.ws.onclose = () => {
            console.log("WebSocket connection to YOLO backend closed.");
            this.stop();
        };

        this.ws.onerror = (error) => {
            console.error("WebSocket error:", error);
            this.stop();
        };
    }

    stop() {
        if (this.animationFrameId) cancelAnimationFrame(this.animationFrameId);
        this.animationFrameId = null;
        if (this.ws && this.ws.readyState === WebSocket.OPEN) this.ws.close();
        console.log("Frame processor stopped.");
    }

    sendFrameLoop = () => {
        if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
        this.ctx.drawImage(this.videoElement, 0, 0, this.canvas.width, this.canvas.height);
        this.ws.send(this.canvas.toDataURL('image/jpeg', 0.7));
        this.animationFrameId = requestAnimationFrame(this.sendFrameLoop);
    }
}