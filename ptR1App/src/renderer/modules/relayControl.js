// toggleRelay(), updateButton(), state cache
let relayState = false;

function updateButton() {
  const btn = document.getElementById("relayButton");
  btn.textContent = `RELAY: ${relayState ? "ON" : "OFF"}`;
  btn.classList.toggle("on",  relayState);
  btn.classList.toggle("off", !relayState);
}

function toggleRelay() {
  relayState = !relayState;
  updateButton();
  window.electronAPI.sendRelayCommand(relayState ? "on" : "off");
}

export function initRelayButtons() {
  document.getElementById("relayButton").addEventListener("click", toggleRelay);
  updateButton();
  console.log("Relay Control: Initialized relay button.");
}