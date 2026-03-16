// modules/recorder.js

export class CanvasRecorder {
  #canvas;
  #mediaRecorder = null;
  #recordedChunks = [];
  #recordingTimeout = null;
  #isRecording = false;

  constructor(canvas, options = {}) {
    this.#canvas = canvas;
    this.fps = options.fps || 15;
    this.segmentMs = options.segmentMs || 10 * 60 * 1000; // default 10 นาที
  }

  start() {
    console.log('Start recording segment...');
    // ถ้ากำลังอัดอยู่แล้ว (และไม่ใช่การ loop ต่อเนื่อง) ให้ข้าม
    if (this.#isRecording && this.#mediaRecorder?.state === 'recording') return;

    const stream = this.#canvas.captureStream(this.fps);
    this.#recordedChunks = [];
    this.#isRecording = true;

    this.#mediaRecorder = new MediaRecorder(stream, { mimeType: 'video/webm' });

    this.#mediaRecorder.ondataavailable = (e) => {
      if (e.data.size > 0) this.#recordedChunks.push(e.data);
    };

    this.#mediaRecorder.onstop = () => this.#handleSegmentComplete();

    this.#mediaRecorder.start();
    
    // ตั้งเวลาสำหรับตัดคลิป
    this.#recordingTimeout = setTimeout(() => {
      if (this.#mediaRecorder?.state === 'recording') {
        console.log('Segment finished (timeout). Stopping...');
        this.#mediaRecorder.stop();
      }
    }, this.segmentMs);
  }

  stop() {
    console.log('Stop recording manually.');
    if (!this.#isRecording) return;

    clearTimeout(this.#recordingTimeout);
    this.#isRecording = false; // ปิด flag เพื่อไม่ให้ loop ต่อ

    if (this.#mediaRecorder?.state === 'recording') {
      this.#mediaRecorder.stop();
    }
  }

  #handleSegmentComplete() {
    const chunksToSave = this.#recordedChunks;  // ← snapshot ก่อน
    this.#recordedChunks = [];                   // ← reset ให้ segment ใหม่

    if (this.#isRecording) {
        this.start();
    }

    const blob = new Blob(chunksToSave, { type: 'video/webm' });

    if (blob.size === 0) {
        console.warn("⚠️ Skipped empty recording (0 byte)");
        return;
    }

    const now = new Date();
    const dateStr = now.toISOString().split('T')[0];
    const timeStr = now.toTimeString().slice(0, 8).replace(/:/g, '-');

    console.log(`Saving video segment (${(blob.size / 1024 / 1024).toFixed(2)} MB)...`);

    blob.arrayBuffer().then((buffer) => {
        window.electronAPI.saveVideo({
            buffer,
            date: dateStr,
            filename: `record-${timeStr}.webm`
        });
    });
  }
}