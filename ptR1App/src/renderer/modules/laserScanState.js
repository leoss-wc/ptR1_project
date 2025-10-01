// modules/laserScanState.js
export let latestScan = null;

export function updateLaserScan(scanData) {
  latestScan = scanData;
}