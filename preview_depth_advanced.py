import cv2
import numpy as np
import ArducamDepthCamera as ac
import os

# ====== ИНИЦИАЛИЗАЦИЯ КАМЕРЫ ======
camera = ac.ArducamCamera()
camera.initialize()
camera.start()

print("✅ Camera started.")
print("Hotkeys:")
print("  q - quit")
print("  s - save combined frame (before/after)")
print("  a - save only filtered frame (after)")
print("  v - start/stop video recording")

# Проверяем guided filter
try:
    import cv2.ximgproc as xip
    guided_filter_available = True
except ImportError:
    guided_filter_available = False
    print("⚠️ Guided filter не найден (установи 'opencv-contrib-python').")

# ====== НАСТРОЙКИ И СЛАЙДЕРЫ ======
def nothing(x): pass

cv2.namedWindow("Controls")
cv2.resizeWindow("Controls", 400, 250)
cv2.createTrackbar("Confidence threshold", "Controls", 30, 100, nothing)
cv2.createTrackbar("Median ksize", "Controls", 5, 15, nothing)
cv2.createTrackbar("Guided radius", "Controls", 8, 30, nothing)
cv2.createTrackbar("Guided eps x100", "Controls", 2, 20, nothing)
cv2.createTrackbar("Morph size", "Controls", 3, 10, nothing)

frame_counter = 0
video_writer = None
video_recording = False

# Папка для сохранений
os.makedirs("saved_frames", exist_ok=True)

# ====== ГЛАВНЫЙ ЦИКЛ ======
while True:
    frame = camera.requestFrame(200)
    if frame is None:
        continue

    depth_buf = frame.getDepthData()
    confidence_buf = frame.getConfidenceData()
    camera.releaseFrame(frame)

    # Преобразуем глубину в 8-бит
    depth_vis = cv2.convertScaleAbs(depth_buf, alpha=255.0 / np.max(depth_buf))
    depth_vis = np.clip(depth_vis, 0, 255).astype(np.uint8)

    # ====== Чтение параметров ======
    CONFIDENCE_THRESHOLD = cv2.getTrackbarPos("Confidence threshold", "Controls")
    MEDIAN_KSIZE = cv2.getTrackbarPos("Median ksize", "Controls")
    if MEDIAN_KSIZE % 2 == 0:
        MEDIAN_KSIZE += 1
    GUIDED_RADIUS = cv2.getTrackbarPos("Guided radius", "Controls")
    GUIDED_EPS = cv2.getTrackbarPos("Guided eps x100", "Controls") / 100.0 * 255 * 255
    MORPH_SIZE = cv2.getTrackbarPos("Morph size", "Controls")
    kernel = np.ones((MORPH_SIZE, MORPH_SIZE), np.uint8)

    # ====== Фильтрация ======
    mask = (confidence_buf > CONFIDENCE_THRESHOLD).astype(np.uint8)
    depth_masked = cv2.bitwise_and(depth_vis, depth_vis, mask=mask)
    depth_median = cv2.medianBlur(depth_masked, MEDIAN_KSIZE)

    if guided_filter_available and GUIDED_RADIUS > 0:
        depth_guided = xip.guidedFilter(
            guide=depth_vis, src=depth_median,
            radius=GUIDED_RADIUS, eps=GUIDED_EPS
        )
    else:
        depth_guided = depth_median

    depth_clean = cv2.morphologyEx(depth_guided, cv2.MORPH_OPEN, kernel)
    depth_clean = cv2.morphologyEx(depth_clean, cv2.MORPH_CLOSE, kernel)

    # ====== Цветные карты ======
    color_before = cv2.applyColorMap(depth_vis, cv2.COLORMAP_RAINBOW)
    color_after = cv2.applyColorMap(depth_clean, cv2.COLORMAP_RAINBOW)

    combined = np.hstack((color_before, color_after))
    cv2.putText(combined, "Before", (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(combined, "After",
                (color_before.shape[1] + 30, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    # ====== Видео запись ======
    if video_recording:
        if video_writer is None:
            h, w, _ = combined.shape
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            video_writer = cv2.VideoWriter("saved_video.avi", fourcc, 20.0, (w, h))
            print("🔴 Recording started: saved_video.avi")
        video_writer.write(combined)
    elif video_writer:
        print("🟢 Recording stopped.")
        video_writer.release()
        video_writer = None

    # ====== Отображение ======
    cv2.imshow("Arducam ToF Depth Denoising (Before / After)", combined)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        # Сохранить оба кадра (before/after)
        frame_path = os.path.join("saved_frames", f"depth_frame_{frame_counter:04d}.png")
        cv2.imwrite(frame_path, combined)
        print(f"💾 Saved combined frame: {frame_path}")
        frame_counter += 1
    elif key == ord('a'):
        # Сохранить только фильтрованный кадр (after)
        frame_path = os.path.join("saved_frames", f"depth_filtered_{frame_counter:04d}.png")
        cv2.imwrite(frame_path, color_after)
        print(f"💾 Saved filtered frame: {frame_path}")
        frame_counter += 1
    elif key == ord('v'):
        video_recording = not video_recording

# ====== Завершение ======
if video_writer:
    video_writer.release()
camera.stop()
camera.close()
cv2.destroyAllWindows()
print("🛑 Camera stopped.")
