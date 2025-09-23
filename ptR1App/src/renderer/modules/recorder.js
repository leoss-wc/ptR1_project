// canvas.captureStream + MediaRecorder loop

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
    console.log('start record');
    if (this.#isRecording) return;

    const stream = this.#canvas.captureStream(this.fps);
    this.#recordedChunks = [];
    this.#isRecording = true;

    this.#mediaRecorder = new MediaRecorder(stream, { mimeType: 'video/webm' });

    this.#mediaRecorder.ondataavailable = (e) => {
      if (e.data.size > 0) this.#recordedChunks.push(e.data);
    };

    this.#mediaRecorder.onstop = () => this.#handleSegmentComplete();

    this.#mediaRecorder.start();
    this.#recordingTimeout = setTimeout(() => {
      if (this.#mediaRecorder?.state === 'recording') {
        this.#mediaRecorder.stop();
      }
    }, this.segmentMs);
  }

  stop() {
    console.log('stop record');

    if (!this.#isRecording) return;

    clearTimeout(this.#recordingTimeout);
    this.#isRecording = false;

    if (this.#mediaRecorder?.state === 'recording') {
      this.#mediaRecorder.stop();
    }
  }

  #handleSegmentComplete() {
    const blob = new Blob(this.#recordedChunks, { type: 'video/webm' });

    if (blob.size === 0) {
      console.warn("⚠️ Skipped empty recording (0 byte)");
      if (this.#isRecording) this.start(); // วนรอบถัดไป
      return;
    }

    const now = new Date();
    const dateStr = now.toISOString().split('T')[0]; // yyyy-mm-dd
    const timeStr = now.toTimeString().slice(0, 5).replace(':', '-'); // hh-mm
    console.log('save video');
    blob.arrayBuffer().then((buffer) => {
      window.electronAPI.saveVideo({
        buffer,
        date: dateStr,
        filename: `record-${timeStr}.webm`
      });

      if (this.#isRecording) this.start(); // 🔁 เริ่มรอบถัดไป
    });
  }
}
