export const SystemMonitor = {
    updateUI(data) {
        if (!data) return;

        // 1. System Overview
        const sys = data.system || {};
        this.setText('sys-cpu-total', `${sys.cpu_total ?? '--'}%`);
        this.setText('sys-ram',       `${sys.ram_percent ?? '--'}%`);
        this.setText('sys-ram-mb',    `(${sys.ram_used_mb ?? '--'} MB)`);

        // อุณหภูมิ — เปลี่ยนสีตามความร้อน
        const tempEl = document.getElementById('sys-temp');
        if (tempEl) {
            const t = sys.temperature ?? 0;
            tempEl.innerText = `${t}°C`;
            tempEl.style.color = t > 70 ? '#ff4444' : t > 55 ? '#ffbb33' : '#00C851';
        }

        // 2. CPU by Services
        const svc = data.cpu_services || {};
        this.setText('sys-svc-streammgr', `${svc.stream_mgr  ?? 0}%`);
        this.setText('sys-svc-ffmpeg',    `${svc.ffmpeg      ?? 0}%`);
        this.setText('sys-svc-movebase',  `${svc.move_base   ?? 0}%`);
        this.setText('sys-svc-amcl',      `${svc.amcl        ?? 0}%`);
        this.setText('sys-svc-gmapping',  `${svc.gmapping    ?? 0}%`);
        this.setText('sys-svc-mapserver', `${svc.map_server  ?? 0}%`);
        this.setText('sys-svc-rosbridge', `${svc.rosbridge   ?? 0}%`);
        this.setText('sys-svc-ydlidar',   `${svc.ydlidar     ?? 0}%`);
        this.setText('sys-svc-rosserial', `${svc.rosserial   ?? 0}%`);
        this.setText('sys-svc-others',    `${svc.others      ?? 0}%`);

        // 3. AI Section
        const ai = data.ai || {};
        const aiEnabledEl = document.getElementById('sys-ai-enabled');
        if (aiEnabledEl) {
            const enabled = ai.enabled ?? false;
            aiEnabledEl.innerText = enabled ? 'ON' : 'OFF';
            aiEnabledEl.style.color = enabled ? '#00C851' : '#ff4444';
        }
        this.setText('sys-ai-mode', ai.mode         ?? '--');
        this.setText('sys-ai-ms',   `${ai.inference_ms ?? '--'} ms`);

        // เปลี่ยนสี inference time
        const msEl = document.getElementById('sys-ai-ms');
        if (msEl && ai.inference_ms) {
            msEl.style.color = ai.inference_ms > 200 ? '#ff4444'
                             : ai.inference_ms > 150 ? '#ffbb33'
                             : '#00C851';
        }
    },

    setText(id, value) {
        const el = document.getElementById(id);
        if (el) el.innerText = value;
    }
};