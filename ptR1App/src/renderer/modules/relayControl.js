//toggleRelay(), updateButton(), state cache

const relayStates = { relay1: false, relay2: false };
const relayIdMap = { relayButton1: 'relay1', relayButton2: 'relay2' };

function updateButton(buttonId) {
  const relayId = relayIdMap[buttonId];
  const btn = document.getElementById(buttonId);
  btn.textContent = `${relayId.toUpperCase()}: ${relayStates[relayId] ? "ON" : "OFF"}`;
  btn.classList.toggle("on",  relayStates[relayId]);
  btn.classList.toggle("off", !relayStates[relayId]);
}

function toggleRelay(buttonId) {
  const relayId = relayIdMap[buttonId];
  relayStates[relayId] = !relayStates[relayId];
  updateButton(buttonId);
  window.electronAPI.sendRelayCommand(relayId, relayStates[relayId] ? "on" : "off");
}

export function initRelayButtons() {
  Object.keys(relayIdMap).forEach((id) => {
    document.getElementById(id).addEventListener("click", () => toggleRelay(id));
    updateButton(id);
  });
}
