// videoPlayer.js
// โมดูลสำหรับจัดการ video-player view

let currentVideoFolder = null;

export function setupVideoPlayer() {
  const selectFolderBtn = document.getElementById('select-folder-btn');
  const videoGallery = document.getElementById("video-gallery");
  const videoPlayer = document.getElementById("video-player");

  // ฟังก์ชันโหลดวิดีโอ
  async function loadVideos(folderPath) {
    if (!folderPath) return;
    currentVideoFolder = folderPath;

    // 1. ดึงรายการวิดีโอ (Recursive)
    const videos = await window.electronAPI.loadVideosFromFolder(folderPath);

    videoGallery.innerHTML = '';

    if (videos.length === 0) {
      videoGallery.innerHTML = '<p style="color: #aaa; text-align: center;">No videos found.</p>';
      return;
    }

    // 2. ✨ จัดกลุ่มวิดีโอตาม "ชื่อโฟลเดอร์วันที่"
    const groupedVideos = {};
    
    videos.forEach(video => {
        // relativePath ตัวอย่าง: "2025-05-04/video-10-00.mp4"
        // เราจะตัดเอาแค่ส่วนหน้า "2025-05-04" มาเป็น Group Name
        const parts = video.relativePath.split(/[/\\]/); // รองรับทั้ง / และ \
        const dateGroup = parts.length > 1 ? parts[0] : 'Unsorted';
        
        if (!groupedVideos[dateGroup]) {
            groupedVideos[dateGroup] = [];
        }
        groupedVideos[dateGroup].push(video);
    });

    // 3. เรียงลำดับวันที่ (ใหม่ -> เก่า)
    const sortedDates = Object.keys(groupedVideos).sort().reverse();

    // 4. วาดลงหน้าจอทีละกลุ่ม
    sortedDates.forEach(date => {
        // สร้างหัวข้อวันที่
        const groupContainer = document.createElement('div');
        groupContainer.className = 'video-group';
        groupContainer.style.marginBottom = '20px';

        const groupTitle = document.createElement('h4');
        groupTitle.textContent = `📅 ${date}`;
        groupTitle.style.color = '#ffa500'; // สีส้ม
        groupTitle.style.borderBottom = '1px solid #444';
        groupTitle.style.paddingBottom = '5px';
        groupTitle.style.marginBottom = '10px';
        groupContainer.appendChild(groupTitle);

        const groupGrid = document.createElement('div');
        groupGrid.className = 'video-grid'; // ใช้ Grid เดิม
        // ปรับ CSS inline นิดหน่อยเพื่อให้ Grid ซ้อนอยู่ใน Group ได้สวย
        groupGrid.style.display = 'grid';
        groupGrid.style.gridTemplateColumns = 'repeat(auto-fill, minmax(120px, 1fr))';
        groupGrid.style.gap = '10px';

        // วาดวิดีโอในกลุ่มนั้น
        groupedVideos[date].forEach(({ relativePath, name }) => {
            // (ใช้ Logic เดิมในการสร้าง Thumbnail)
            window.electronAPI.getVideoFileURL(relativePath).then(videoSrc => {
                const thumbWrapper = document.createElement('div');
                thumbWrapper.style.cursor = 'pointer';

                const thumb = document.createElement("video");
                thumb.src = videoSrc;
                thumb.className = "video-thumb";
                thumb.muted = true;
                thumb.style.width = '100%';
                thumb.style.borderRadius = '5px';
                
                // Play on Hover
                thumb.addEventListener('mouseenter', () => thumb.play().catch(() => {}));
                thumb.addEventListener('mouseleave', () => { thumb.pause(); thumb.currentTime = 0; });

                // Click to Play
                thumb.addEventListener("click", () => {
                    console.log('🎥 Playing:', name);
                    videoPlayer.src = videoSrc;
                    videoPlayer.load();
                    videoPlayer.play().catch((err) => console.warn(err));
                });

                // ชื่อไฟล์ใต้คลิป
                const label = document.createElement('div');
                label.textContent = name; // หรือแสดงแค่เวลาถ้าชื่อยาวไป
                label.style.fontSize = '10px';
                label.style.color = '#ccc';
                label.style.marginTop = '2px';
                label.style.overflow = 'hidden';
                label.style.textOverflow = 'ellipsis';
                label.style.whiteSpace = 'nowrap';

                thumbWrapper.appendChild(thumb);
                thumbWrapper.appendChild(label);
                groupGrid.appendChild(thumbWrapper);
            });
        });

        groupContainer.appendChild(groupGrid);
        videoGallery.appendChild(groupContainer);
    });
  }

  // ปุ่มเลือกโฟลเดอร์
  selectFolderBtn.addEventListener("click", async () => {
    const folderPath = await window.electronAPI.selectFolder_video();
    if (folderPath) {
      await loadVideos(folderPath);
    }
  });

  // ฟังก์ชันโหลด Default
  async function loadDefaultVideos() {
    try {
      const defaultPath = await window.electronAPI.getDefaultVideoPath();
      if (defaultPath) {
        await loadVideos(defaultPath);
      }
    } catch (err) {
      console.error('Failed to load default videos:', err);
    }
  }

  // Listener สำหรับ Refresh อัตโนมัติหลัง Save
  if (window.electronAPI.onVideoSaveStatus) {
      window.electronAPI.onVideoSaveStatus((result) => {
        if (result.success) {
            // alert(`✅ Saved: ${result.path}`); // ปิด Alert ก็ได้ถ้ารำคาญ
            if (currentVideoFolder) loadVideos(currentVideoFolder);
            else loadDefaultVideos();
        } else {
            alert(`⚠️ Error: ${result.message}`);
        }
      });
  }

  loadDefaultVideos();
}