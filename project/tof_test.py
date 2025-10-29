import cv2
import numpy as np
from tof_camera import ToFCamera

from sklearn.cluster import KMeans

def denoise_gray_image(image):
    # Проверяем, что изображение корректное
    if image is None or len(image.shape) != 2 or image.dtype != np.uint8:
        raise ValueError("Ожидается одноканальное 8-битное изображение (np.uint8)")

    image[image == 0] = 255

    # 1. Медианный фильтр — эффективно удаляет точечный шум
    denoised = cv2.medianBlur(image, 5)

    # 2. Билатеральный фильтр — сохраняет границы при сглаживании шумов
    denoised = cv2.bilateralFilter(denoised, d=7, sigmaColor=75, sigmaSpace=75)

    # 3. Морфологическая операция "открытие" — убирает мелкие пятна
    kernel = np.ones((3, 3), np.uint8)
    denoised = cv2.morphologyEx(denoised, cv2.MORPH_OPEN, kernel)

    return denoised

def apply_mask_to_color_image(gray_image, color_image, min_val, max_val):
    # Проверка изображений
    if gray_image is None or len(gray_image.shape) != 2:
        raise ValueError("gray_image должен быть одноканальным")
    if color_image is None or len(color_image.shape) != 3 or color_image.shape[2] != 3:
        raise ValueError("color_image должен быть 3-канальным")
    if gray_image.shape != color_image.shape[:2]:
        raise ValueError("gray_image и color_image должны иметь одинаковую ширину и высоту")

    # Создаем маску (True где значение в диапазоне, False где нет)
    mask = (gray_image >= min_val) & (gray_image <= max_val)

    # Конвертируем маску в тип uint8 (0 или 255)
    mask_uint8 = mask.astype(np.uint8) * 255

    # Накладываем маску на цветное изображение
    masked_color = cv2.bitwise_and(color_image, color_image, mask=mask_uint8)

    return masked_color

def apply_mask_with_gray_background(gray_image, color_image, min_val, max_val):

    # Проверка изображений
    if gray_image is None or len(gray_image.shape) != 2:
        raise ValueError("gray_image должен быть одноканальным")
    if color_image is None or len(color_image.shape) != 3 or color_image.shape[2] != 3:
        raise ValueError("color_image должен быть 3-канальным")
    if gray_image.shape != color_image.shape[:2]:
        raise ValueError("gray_image и color_image должны иметь одинаковую ширину и высоту")

    # Создаем маску (True где значение в диапазоне)
    mask = (gray_image >= min_val) & (gray_image <= max_val)

    # Конвертируем маску в uint8 для работы с OpenCV
    mask_uint8 = mask.astype(np.uint8) * 255

    # Создаем серую версию цветного изображения
    gray_background = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    gray_background = cv2.cvtColor(gray_background, cv2.COLOR_GRAY2BGR)

    # Объединяем: цвет сохраняем там, где маска True, остальное серое
    result = np.where(mask[:, :, None], color_image, gray_background)

    return result

def fast_limit_colors(img: np.ndarray, num_colors: int) -> np.ndarray:
    # --- Input checks ---
    if not isinstance(img, np.ndarray):
        raise TypeError("Input must be a numpy ndarray.")
    if img.dtype != np.uint8:
        raise TypeError("Image must be 8-bit (dtype=np.uint8).")
    if img.ndim != 3 or img.shape[2] != 3:
        raise ValueError("Image must be 3-channel (RGB/BGR).")
    if num_colors < 1 or num_colors > 256**3:
        raise ValueError("num_colors must be between 1 and 16,777,216.")

    # --- Calculate per-channel levels ---
    # Approximate cube root to split colors evenly across 3 channels
    levels_per_channel = int(round(num_colors ** (1/3)))
    factor = 256 // levels_per_channel

    # --- Quantize image ---
    reduced_img = (img // factor) * factor + factor // 2  # center bins

    return reduced_img

def fast_limit_colors(img: np.ndarray, num_colors: int) -> np.ndarray:
    # --- Input checks ---
    if not isinstance(img, np.ndarray):
        raise TypeError("Input must be a numpy ndarray.")
    if img.dtype != np.uint8:
        raise TypeError("Image must be 8-bit (dtype=np.uint8).")
    if img.ndim != 3 or img.shape[2] != 3:
        raise ValueError("Image must be 3-channel (RGB/BGR).")
    if num_colors < 1 or num_colors > 256**3:
        raise ValueError("num_colors must be between 1 and 16,777,216.")

    # --- Calculate per-channel levels ---
    # Approximate cube root to split colors evenly across 3 channels
    levels_per_channel = int(round(num_colors ** (1/3)))
    factor = 256 // levels_per_channel

    # --- Quantize image ---
    reduced_img = (img // factor) * factor + factor // 2  # center bins

    return reduced_img

def hybrid_limit_colors(img: np.ndarray, num_colors: int = 5, pre_reduce_factor: int = 4) -> np.ndarray:
    # --- Input checks ---
    if not isinstance(img, np.ndarray):
        raise TypeError("Input must be a numpy ndarray.")
    if img.dtype != np.uint8:
        raise TypeError("Image must be 8-bit (dtype=np.uint8).")
    if img.ndim != 3 or img.shape[2] != 3:
        raise ValueError("Image must be 3-channel (RGB/BGR).")
    if num_colors < 1 or num_colors > 256:
        raise ValueError("num_colors must be between 1 and 256.")

    # --- Step 1: Fast pre-quantization ---
    levels_per_channel = int(round((num_colors * pre_reduce_factor) ** (1/3)))
    factor = max(1, 256 // levels_per_channel)
    pre_quantized = (img // factor) * factor + factor // 2

    # --- Step 2: Reshape and KMeans ---
    pixels = pre_quantized.reshape(-1, 3)
    kmeans = KMeans(n_clusters=num_colors, random_state=42, n_init=10)
    kmeans.fit(pixels)
    new_colors = kmeans.cluster_centers_.astype(np.uint8)
    labels = kmeans.labels_

    # --- Step 3: Reconstruct image ---
    new_img = new_colors[labels].reshape(img.shape)
    return new_img

def find_closest_object(depth_img, threshold_ratio=0.2):
    """
    Находит самый близкий объект на изображении глубины.
    
    depth_img: одноканальное 8-битное изображение глубины
    threshold_ratio: верхний процент яркости для выделения объектов
    """
    if len(depth_img.shape) != 2 or depth_img.dtype != np.uint8:
        raise ValueError("Изображение должно быть одноканальным 8-битным")

    # Игнорируем черные пиксели (значение 0)
    non_zero = depth_img[depth_img > 0]
    if non_zero.size == 0:
        return None  # нет объектов

    # Определяем порог
    max_val = non_zero.max()
    min_val = non_zero.min()
    threshold_val = max_val - int((max_val - min_val) * threshold_ratio)

    _, binary = cv2.threshold(depth_img, threshold_val, 255, cv2.THRESH_BINARY)

    # Находим контуры
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None  # объекты не найдены

    # Находим контур с максимальной средней яркостью
    max_mean = -1
    closest_contour = None
    for cnt in contours:
        mask = np.zeros_like(depth_img)
        cv2.drawContours(mask, [cnt], -1, 255, -1)
        mean_val = cv2.mean(depth_img, mask=mask)[0]
        if mean_val > max_mean:
            max_mean = mean_val
            closest_contour = cnt

    # Возвращаем координаты bounding box ближайшего объекта
    x, y, w, h = cv2.boundingRect(closest_contour)
    return (x, y, w, h), closest_contour

def main():
    # Инициализация камеры
    cam = ToFCamera(max_distance=4000, confidence_threshold=30)

    print(f"Camera resolution: {cam.width}x{cam.height}")
    print("Press 'q' to quit, '+' / '-' to change confidence threshold")

    try:
        while True:
            # Получаем отфильтрованные кадры
            frame = cam.get_filtered_frame()
            if frame is None:
                continue

            depth = frame["depth"]
            confidence = frame["confidence"]
            amplitude = frame["amplitude"]
            
            depth = 255 - denoise_gray_image(depth)
            depth[depth < 128] = 0

            # Нормализуем для отображения
            confidence_vis = cv2.normalize(confidence, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            amplitude_vis = cv2.normalize(amplitude, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

            # Применяем цветовую карту к глубине
            depth_colored = cv2.applyColorMap(depth, cv2.COLORMAP_RAINBOW)

            #depth_colored = apply_mask_to_color_image(depth, depth_colored, min_val=20, max_val=180)
            #depth_colored = hybrid_limit_colors(depth_colored)

            # Отображаем три окна
            cv2.imshow("Depth (filtered)", depth)
            cv2.imshow("Depth colored (filtered)", depth_colored)
            cv2.imshow("Confidence", confidence_vis)
            cv2.imshow("Amplitude", amplitude_vis)
            
            result = cam.find_closest_object()
            if result:
                (x, y, w, h), contour = result
                print("Ближайший объект:", x, y, w, h)
                output = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
                cv2.rectangle(output, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.imshow("Closest Object", output)
            else:
                print("Объекты не найдены")
                cv2.destroyWindow("Closest Object")

            # Управление с клавиатуры
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('+'):
                cam.confidence_threshold = min(cam.confidence_threshold + 5, 255)
                print("Confidence threshold:", cam.confidence_threshold)
            elif key == ord('-'):
                cam.confidence_threshold = max(cam.confidence_threshold - 5, 0)
                print("Confidence threshold:", cam.confidence_threshold)

    except KeyboardInterrupt:
        pass
    finally:
        cam.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
