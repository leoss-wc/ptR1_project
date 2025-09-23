// videoPlayer.js
// โมดูลสำหรับจัดการ video-player view

let currentVideoFolder = null;
export function setupVideoPlayer() {
  const selectFolderBtn = document.getElementById('select-folder-btn');
  const videoGallery = document.getElementById("video-gallery");
  const videoPlayer = document.getElementById("video-player");

  async function loadVideos(folderPath) {
    if (!folderPath) return;
    currentVideoFolder = folderPath;

    // ดึงรายการวิดีโอจาก API
    const videos = await window.electronAPI.loadVideosFromFolder(folderPath);

    videoGallery.innerHTML = '';

    for (const { relativePath } of videos) {
      const videoSrc = await window.electronAPI.getVideoFileURL(relativePath);

      const thumb = document.createElement("video");
      thumb.src = videoSrc;
      thumb.className = "video-thumb";
      thumb.muted = true;
      thumb.loop = true;

      thumb.addEventListener("click", () => {
        const source = videoPlayer.querySelector("source");
        source.src = videoSrc;
        videoPlayer.load();
        console.log('🎥 Playing from:', videoSrc);

        videoPlayer.onloadeddata = () => {
          videoPlayer.play().catch((err) => {
            console.warn("🎥 Video play interrupted:", err.message);
          });
        };
      });

      videoGallery.appendChild(thumb);
    }
  }

  selectFolderBtn.addEventListener("click", async () => {
    const folderPath = await window.electronAPI.selectFolder_video();
    if (folderPath) {
      await loadVideos(folderPath);
    }
  });

  // ฟังก์ชันสำหรับโหลดวิดีโอเริ่มต้น
  async function loadDefaultVideos() {
    try {
      const defaultPath = await window.electronAPI.getDefaultVideoPath();
      if (defaultPath) {
        console.log(`🎥 Loading default videos from: ${defaultPath}`);
        await loadVideos(defaultPath);
      }
    } catch (err) {
      console.error('Failed to load default videos:', err);
    }
  }

  // เรียกใช้งานฟังก์ชันนี้ทันทีที่โมดูลถูก setup
  loadDefaultVideos();
}
