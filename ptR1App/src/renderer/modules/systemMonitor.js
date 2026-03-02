// systemMonitor.js
export const SystemMonitor = {
    updateUI(data) {
        if (!data) return;

        // 1. อัปเดตข้อมูล System หลัก
        const sys = data.system || {};
        this.setText('sys-cpu-total', `${sys.cpu_total}%`);
        this.setText('sys-ram', `${sys.ram_percent}%`);
        this.setText('sys-temp', `${sys.temperature}°C`);

        // เปลี่ยนสีอุณหภูมิตามความร้อน
        const tempEl = document.getElementById('sys-temp');
        if (tempEl) {
            tempEl.style.color = sys.temperature > 70 ? '#ff4444' : (sys.temperature > 55 ? '#ffbb33' : '#00C851');
        }

        // 2. อัปเดตข้อมูลราย Service
        const svc = data.cpu_services || {};
        this.setText('sys-svc-movebase', `${svc.move_base}%`);
        this.setText('sys-svc-gmapping', `${svc.gmapping}%`);
        this.setText('sys-svc-rosserial', `${svc.rosserial}%`);
        this.setText('sys-svc-ydlidar', `${svc.ydlidar}%`);
        this.setText('sys-svc-rosbridge', `${svc.rosbridge}%`);
        this.setText('sys-svc-others', `${svc.others}%`);
    },

    setText(id, value) {
        const el = document.getElementById(id);
        if (el) el.innerText = value;
    }
};