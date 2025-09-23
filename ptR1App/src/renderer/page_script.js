
  const items = document.querySelectorAll(".sidebar-item");
  const views = document.querySelectorAll(".view");
  
  items.forEach(item => {
    item.addEventListener("click", () => {
      items.forEach(i => i.classList.remove("active"));
      views.forEach(v => v.classList.add("hidden"));
      item.classList.add("active");
      document.getElementById("view-" + item.dataset.view).classList.remove("hidden");
    });
  });

const pwmSlider = document.getElementById('pwm-slider');
const pwmValueLabel = document.getElementById('pwm-value-label');

if (pwmSlider && pwmValueLabel) {
  pwmSlider.addEventListener('input', () => {
    pwmValueLabel.textContent = pwmSlider.value;
  });
}

let isDrawMode = false;

document.getElementById('toggle-draw-mode').addEventListener('click', () => {
  isDrawMode = !isDrawMode;
  document.getElementById('toggle-draw-mode').textContent = `Draw : ${isDrawMode ? 'ON' : 'OFF'}`;
});

document.getElementById('btn-static-map').addEventListener('click', () => {
  document.getElementById('staticMapCanvas').classList.remove('hidden');
  document.getElementById('liveMapCanvas').classList.add('hidden');

  document.getElementById('btn-static-map').classList.add('active');
  document.getElementById('btn-live-map').classList.remove('active');

  document.getElementById('static-control-box').classList.remove('hidden');
  document.getElementById('live-control-box').classList.add('hidden');

  document.getElementById('patrol-status-label').classList.remove('hidden');

});

document.getElementById('btn-live-map').addEventListener('click', () => {
  document.getElementById('staticMapCanvas').classList.add('hidden');
  document.getElementById('liveMapCanvas').classList.remove('hidden');

  document.getElementById('btn-static-map').classList.remove('active');
  document.getElementById('btn-live-map').classList.add('active');

  document.getElementById('static-control-box').classList.add('hidden');
  document.getElementById('live-control-box').classList.remove('hidden');

  document.getElementById('patrol-status-label').classList.add('hidden');


});

  





 
  
  