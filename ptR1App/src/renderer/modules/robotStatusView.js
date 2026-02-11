// src/renderer/js/modules/robot-status.js

export class RobotStatusRenderer {
    constructor() {
        // Cache Elements ไว้เลย ไม่ต้องค้นหาใหม่ทุกครั้งที่ข้อมูลมา
        this.elMode = document.getElementById('mon-mode');
        this.elWd = document.getElementById('mon-wd');
        this.elWdTime = document.getElementById('mon-wd-time');
        this.elPid = document.getElementById('mon-pid');
        this.elBat = document.getElementById('mon-bat');
        this.elRelay = document.getElementById('mon-relay');
    }

    // ฟังก์ชันนี้จะถูกเรียกโดย app.js เมื่อมีข้อมูลมา
    update(str) {
        if (!str) return;

        try {
            // 1. Mode
            const modeMatch = str.match(/\[(.*?)\]/);
            if (modeMatch && this.elMode) {
                this.elMode.innerText = modeMatch[1];
                this.elMode.style.color = (modeMatch[1].includes('MAN')) ? '#ffbb33' : '#00C851';
            }

            // 2. Watchdog
            const wdMatch = str.match(/WD:(.*?)\((\d+)ms\)/);
            if (wdMatch && this.elWd) {
                const status = wdMatch[1];
                this.elWd.innerText = status;
                if(this.elWdTime) this.elWdTime.innerText = `(${wdMatch[2]}ms)`;
                this.elWd.style.color = (status === 'OK') ? '#00C851' : '#ff4444';
            }

            // 3. PID
            const pidMatch = str.match(/PID:([\d\.]+),([\d\.]+),([\d\.]+)/);
            if (pidMatch && this.elPid) {
                this.elPid.innerText = `P:${pidMatch[1]}  I:${pidMatch[2]}  D:${pidMatch[3]}`;
            }

            // 4. Battery & Relay
            const batMatch = str.match(/Bat:([\d\.]+)V/);
            if (batMatch && this.elBat) this.elBat.innerText = batMatch[1] + " V";

            const rMatch = str.match(/R:(\d+),(\d+)/);
            if (rMatch && this.elRelay) this.elRelay.innerText = `R1:${rMatch[1]} R2:${rMatch[2]}`;

        } catch (err) {
            console.error("Error parsing robot status:", err);
        }
    }
}

export class PidTuner {
    constructor() {
        this.inputKp = document.getElementById('input-kp');
        this.inputKi = document.getElementById('input-ki');
        this.inputKd = document.getElementById('input-kd');
        this.btnUpdate = document.getElementById('btn-update-pid');

        this.isUserTyping = false; // ป้องกันเลขเด้งตอนกำลังพิมพ์

        this.init();
    }

    init() {
        if (this.btnUpdate) {
            this.btnUpdate.addEventListener('click', () => this.sendPidCommand());
        }

        // เช็คว่า User กำลังพิมพ์ไหม (ถ้าพิมพ์อยู่ อย่าเพิ่งเอาค่าจากหุ่นมาทับ)
        [this.inputKp, this.inputKi, this.inputKd].forEach(el => {
            if(el) {
                el.addEventListener('focus', () => this.isUserTyping = true);
                el.addEventListener('blur', () => this.isUserTyping = false);
            }
        });
    }

    sendPidCommand() {
        const p = this.inputKp.value || 0;
        const i = this.inputKi.value || 0;
        const d = this.inputKd.value || 0;

        // สร้าง Command String ตาม Format ที่ C++ รอรับ
        // ตัวอย่าง: "set_pid:1.5,5.0,0.05"
        const commandString = `set_pid:${p},${i},${d}`;
        
        console.log("Sending:", commandString);

        if (window.api && window.api.sendCommand) {
            window.api.sendCommand(commandString);
        }
    }
    // ฟังก์ชันนี้จะรับ String จาก Robot Status มาอัปเดตใส่ช่อง Input
    // เพื่อให้เรารู้ว่าค่าปัจจุบันบนบอร์ดจริงๆ คือเท่าไหร่
    updateFromStatus(str) {
        if (this.isUserTyping) return; // ถ้าพิมพ์อยู่ ไม่ต้องอัปเดต

        // Regex หาค่า PID: P, I, D
        const match = str.match(/PID:([\d\.]+),([\d\.]+),([\d\.]+)/);
        if (match) {
            if (this.inputKp) this.inputKp.value = parseFloat(match[1]);
            if (this.inputKi) this.inputKi.value = parseFloat(match[2]);
            if (this.inputKd) this.inputKd.value = parseFloat(match[3]);
        }
    }
}