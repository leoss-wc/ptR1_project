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
        console.log("RobotStatusRenderer initialized.");
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
            if (batMatch && this.elBat) {
                const voltage = parseFloat(batMatch[1]); // แปลง String เป็น Float
                const percent = this.getBatteryPercent(voltage); // คำนวณ %
                
                // แสดงผล: "12.5 V (85%)"
                this.elBat.innerText = `${percent}% (${voltage.toFixed(2)} V)`;
                
                // เปลี่ยนสีตามระดับแบตเตอรี่
                if (percent <= 20) {
                    this.elBat.style.color = '#ff4444'; // แดง (ต่ำ)
                } else if (percent <= 50) {
                    this.elBat.style.color = '#ffbb33'; // เหลือง (กลาง)
                } else {
                    this.elBat.style.color = '#00C851'; // เขียว (สูง)
                }
            }

            const rMatch = str.match(/R:(\d+),(\d+)/);
            if (rMatch && this.elRelay) this.elRelay.innerText = `R1:${rMatch[1]} R2:${rMatch[2]}`;

        } catch (err) {
            console.error("Error parsing robot status:", err);
        }

    }
    // --- ฟังก์ชันคำนวณ % สำหรับ LiFePO4 4S โดยเฉพาะ ---
    getBatteryPercent(voltage) {
        // LiFePO4 กราฟไม่เป็นเส้นตรง (Non-linear)
        // ช่วง 13.0V - 13.4V แบตจะอยู่นานมาก (คือช่วง 30% - 90%)
        // ต่ำกว่า 12.0V คือร่วงเร็วมาก
        
        // กำหนดจุดช่วงแรงดัน (Voltage Points)
        // > 13.40V = 100% (เต็ม)
        //   13.20V = 70%
        //   13.00V = 40%
        //   12.80V = 20% (เริ่มเตือน)
        // < 12.00V = 0% (ควรชาร์จทันที)

        let pct = 0;

        if (voltage >= 13.40) {
            // ช่วงเต็ม (13.4 - 14.6V)
            pct = 100; 
        } else if (voltage >= 13.20) {
            // ช่วง 70% - 100% (Linear mapping ในช่วงสั้นๆ)
            pct = 70 + ((voltage - 13.20) / (13.40 - 13.20) * 30);
        } else if (voltage >= 12.90) {
            // ช่วง 30% - 70% (ช่วงใช้งานปกติ)
            pct = 30 + ((voltage - 12.90) / (13.20 - 12.90) * 40);
        } else if (voltage >= 12.00) {
            // ช่วง 0% - 30% (ช่วงแบตอ่อน แรงดันเริ่มตกไว)
            pct = ((voltage - 12.00) / (12.90 - 12.00) * 30);
        } else {
            // ต่ำกว่า 12V ถือว่าหมด
            pct = 0;
        }
        
        return Math.floor(pct);
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
        console.log("PidTuner initialized.");
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

        if (window.electronAPI && window.electronAPI.sendCommand) {
            window.electronAPI.sendCommand(commandString);
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