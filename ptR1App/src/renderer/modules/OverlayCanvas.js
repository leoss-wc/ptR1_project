export class OverlayCanvas {
    constructor(canvasId, videoElement) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.videoElement = videoElement;
        this.resize();
    }

    resize() {
        this.canvas.width = this.videoElement.clientWidth;
        this.canvas.height = this.videoElement.clientHeight;
    }

    clear() {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    }

    drawDetections(detections) {
        this.clear();
        if (!detections || detections.length === 0) return;

        // คำนวณอัตราส่วนเพื่อวาดบนหน้าจอให้ถูกต้อง
        const scaleX = this.canvas.width / this.videoElement.videoWidth;
        const scaleY = this.canvas.height / this.videoElement.videoHeight;

        detections.forEach(det => {
            const [x1, y1, x2, y2] = det.box;
            const label = `${det.class} ${det.confidence.toFixed(2)}`;
            
            // วาดกรอบ
            this.ctx.strokeStyle = 'lime';
            this.ctx.lineWidth = 2;
            this.ctx.strokeRect(x1 * scaleX, y1 * scaleY, (x2 - x1) * scaleX, (y2 - y1) * scaleY);

            // วาด Label
            this.ctx.fillStyle = 'lime';
            this.ctx.font = '14px sans-serif';
            this.ctx.fillText(label, x1 * scaleX, y1 * scaleY - 5);
        });
    }
}