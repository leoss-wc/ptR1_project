// src/renderer/js/modules/robot-status.js

export class RobotStatusRenderer {
    constructor() {
        this.elMode = document.getElementById('mon-mode');
        this.elWd = document.getElementById('mon-wd');
        this.elWdTime = document.getElementById('mon-wd-time');
        this.elPid = document.getElementById('mon-pid');
        this.elBat = document.getElementById('mon-bat');
        this.elRelay = document.getElementById('mon-relay');
        console.log("RobotStatusRenderer initialized.");
    }

    update(str) {
        if (!str) return;
        try {
            const modeMatch = str.match(/\[(.*?)\]/);
            if (modeMatch && this.elMode) {
                this.elMode.innerText = modeMatch[1];
                this.elMode.style.color = (modeMatch[1].includes('MAN')) ? '#ffbb33' : '#00C851';
            }

            const wdMatch = str.match(/WD:(.*?)\((\d+)ms\)/);
            if (wdMatch && this.elWd) {
                const status = wdMatch[1];
                this.elWd.innerText = status;
                if(this.elWdTime) this.elWdTime.innerText = `(${wdMatch[2]}ms)`;
                this.elWd.style.color = (status === 'OK') ? '#00C851' : '#ff4444';
            }

            const pidMatch = str.match(/PID:([\d\.]+),([\d\.]+),([\d\.]+)/);
            if (pidMatch && this.elPid) {
                this.elPid.innerText = `P:${pidMatch[1]}  I:${pidMatch[2]}  D:${pidMatch[3]}`;
            }

            const batMatch = str.match(/Bat:([\d\.]+)V/);
            if (batMatch && this.elBat) {
                const voltage = parseFloat(batMatch[1]);
                const percent = this.getBatteryPercent(voltage);
                this.elBat.innerText = `${percent}% (${voltage.toFixed(2)} V)`;
                if (percent <= 20) this.elBat.style.color = '#ff4444';
                else if (percent <= 50) this.elBat.style.color = '#ffbb33';
                else this.elBat.style.color = '#00C851';
            }

            const rMatch = str.match(/R:(\d+),(\d+)/);
            if (rMatch && this.elRelay) this.elRelay.innerText = `R1:${rMatch[1]} R2:${rMatch[2]}`;
        } catch (err) {
            console.error("Error parsing robot status:", err);
        }
    }

    getBatteryPercent(voltage) {
        let pct = 0;
        if (voltage >= 13.40) pct = 100; 
        else if (voltage >= 13.20) pct = 70 + ((voltage - 13.20) / (13.40 - 13.20) * 30);
        else if (voltage >= 12.90) pct = 30 + ((voltage - 12.90) / (13.20 - 12.90) * 40);
        else if (voltage >= 12.00) pct = ((voltage - 12.00) / (12.90 - 12.00) * 30);
        else pct = 0;
        return Math.floor(pct);
    }
}

export class PiSystemRenderer {
    constructor() {
        // System Summary
        this.elCpuTotal = document.getElementById('sys-cpu-total');
        this.elRam     = document.getElementById('sys-ram');
        this.elRamMb   = document.getElementById('sys-ram-mb');
        this.elTemp    = document.getElementById('sys-temp');

        // CPU Services
        this.elSvcStreamMgr = document.getElementById('sys-svc-streammgr');
        this.elSvcFfmpeg    = document.getElementById('sys-svc-ffmpeg');
        this.elSvcMoveBase  = document.getElementById('sys-svc-movebase');
        this.elSvcAmcl      = document.getElementById('sys-svc-amcl');
        this.elSvcGmapping  = document.getElementById('sys-svc-gmapping');
        this.elSvcMapServer = document.getElementById('sys-svc-mapserver');
        this.elSvcRosbridge = document.getElementById('sys-svc-rosbridge');
        this.elSvcYdlidar   = document.getElementById('sys-svc-ydlidar');
        this.elSvcRosserial = document.getElementById('sys-svc-rosserial');
        this.elSvcOthers    = document.getElementById('sys-svc-others');

        // AI Section — Model 1 (Person)
        this.elAi1Enabled = document.getElementById('sys-ai1-enabled');
        this.elAi1Mode    = document.getElementById('sys-ai1-mode');
        this.elAi1Ms      = document.getElementById('sys-ai1-ms');

        // AI Section — Model 2 (Door)
        this.elAi2Enabled = document.getElementById('sys-ai2-enabled');
        this.elAi2Mode    = document.getElementById('sys-ai2-mode');
        this.elAi2Ms      = document.getElementById('sys-ai2-ms');

        console.log("PiSystemRenderer initialized.");
    }

    update(data) {
        if (!data) return;

        // 1. System Summary
        const sys = data.system || {};
        if (this.elCpuTotal) this.elCpuTotal.innerText = `${sys.cpu_total ?? '--'}%`;
        if (this.elRam)      this.elRam.innerText      = `${sys.ram_percent ?? '--'}%`;
        if (this.elRamMb)    this.elRamMb.innerText    = `(${sys.ram_used_mb ?? '--'} MB)`;

        if (this.elTemp) {
            const t = sys.temperature ?? 0;
            this.elTemp.innerText   = `${t}°C`;
            this.elTemp.style.color = t > 70 ? '#ff4444' : t > 55 ? '#ffbb33' : '#00C851';
        }

        // 2. CPU Services
        const svc = data.cpu_services || {};
        if (this.elSvcStreamMgr) this.elSvcStreamMgr.innerText = `${svc.stream_mgr  ?? 0}%`;
        if (this.elSvcFfmpeg)    this.elSvcFfmpeg.innerText    = `${svc.ffmpeg      ?? 0}%`;
        if (this.elSvcMoveBase)  this.elSvcMoveBase.innerText  = `${svc.move_base   ?? 0}%`;
        if (this.elSvcAmcl)      this.elSvcAmcl.innerText      = `${svc.amcl        ?? 0}%`;
        if (this.elSvcGmapping)  this.elSvcGmapping.innerText  = `${svc.gmapping    ?? 0}%`;
        if (this.elSvcMapServer) this.elSvcMapServer.innerText = `${svc.map_server  ?? 0}%`;
        if (this.elSvcRosbridge) this.elSvcRosbridge.innerText = `${svc.rosbridge   ?? 0}%`;
        if (this.elSvcYdlidar)   this.elSvcYdlidar.innerText   = `${svc.ydlidar     ?? 0}%`;
        if (this.elSvcRosserial) this.elSvcRosserial.innerText = `${svc.rosserial   ?? 0}%`;
        if (this.elSvcOthers)    this.elSvcOthers.innerText    = `${svc.others      ?? 0}%`;

        // 3. AI Section — enabled/mode ใช้ร่วมกัน, inference_ms แยกต่อโมเดล
        const ai = data.ai || {};
        const enabled = ai.enabled ?? false;
        const mode    = ai.mode ?? '--';

        // Model 1 (Person/COCO)
        if (this.elAi1Enabled) {
            this.elAi1Enabled.innerText   = enabled ? 'ON' : 'OFF';
            this.elAi1Enabled.style.color = enabled ? '#00C851' : '#ff4444';
        }
        if (this.elAi1Mode) this.elAi1Mode.innerText = mode;
        if (this.elAi1Ms) {
            const ms = ai.model1?.inference_ms ?? null;
            this.elAi1Ms.innerText   = ms !== null ? `${ms} ms` : '-- ms';
            this.elAi1Ms.style.color = ms !== null ? '#f0f0f0' : '#ccc';
        }

        // Model 2 (Door)
        if (this.elAi2Enabled) {
            this.elAi2Enabled.innerText   = enabled ? 'ON' : 'OFF';
            this.elAi2Enabled.style.color = enabled ? '#00C851' : '#ff4444';
        }
        if (this.elAi2Mode) this.elAi2Mode.innerText = 'door';
        if (this.elAi2Ms) {
            const ms2 = ai.model2?.inference_ms ?? null;
            this.elAi2Ms.innerText   = ms2 !== null ? `${ms2} ms` : '-- ms';
            this.elAi2Ms.style.color = ms2 !== null ? '#f0f0f0' : '#ccc';
        }
    }
}

export class PidTuner {
    constructor() {
        this.inputKp = document.getElementById('input-kp');
        this.inputKi = document.getElementById('input-ki');
        this.inputKd = document.getElementById('input-kd');
        this.btnUpdate = document.getElementById('btn-update-pid');
        this.isUserTyping = false;
        this.init();
        console.log("PidTuner initialized.");
    }

    init() {
        if (this.btnUpdate) {
            this.btnUpdate.addEventListener('click', () => this.sendPidCommand());
        }
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
        const commandString = `set_pid:${p},${i},${d}`;
        console.log("Sending:", commandString);
        if (window.electronAPI && window.electronAPI.sendCommand) {
            window.electronAPI.sendCommand(commandString);
        }
    }

    updateFromStatus(str) {
        if (this.isUserTyping) return;
        const match = str.match(/PID:([\d\.]+),([\d\.]+),([\d\.]+)/);
        if (match) {
            if (this.inputKp) this.inputKp.value = parseFloat(match[1]);
            if (this.inputKi) this.inputKi.value = parseFloat(match[2]);
            if (this.inputKd) this.inputKd.value = parseFloat(match[3]);
        }
    }
}