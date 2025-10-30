import cv2
import numpy as np

def combine_images(
    images,
    layout='horizontal',
    grid_size=None,
    spacing=5,
    bg_color=(0, 0, 0),
    captions=None,
    font_scale=0.5,
    font_color=(255, 255, 255),
    font_thickness=1,
    caption_height=25,
    border_colors=(255, 255, 255),
    border_thickness=2
):
    """
    Универсальный метод для объединения нескольких изображений NumPy в одно.
    Поддерживает подписи и индивидуальные рамки.
    
    :param images: список numpy-изображений
    :param layout: 'horizontal', 'vertical' или 'grid'
    :param grid_size: (rows, cols) — используется только для 'grid'
    :param spacing: отступ между изображениями (пиксели)
    :param bg_color: цвет фона (B, G, R)
    :param captions: список подписей под изображениями (или None)
    :param font_scale: размер шрифта для подписей
    :param font_color: цвет шрифта (B, G, R)
    :param font_thickness: толщина шрифта
    :param caption_height: высота области для подписи
    :param border_colors: цвет рамки или список цветов [(B,G,R), ...] для каждого изображения
    :param border_thickness: толщина рамки (пиксели)
    :return: объединённое изображение NumPy
    """
    if not images:
        raise ValueError("Список изображений пуст.")

    # Если передан один цвет рамки, применяем его ко всем
    if isinstance(border_colors, tuple) and len(border_colors) == 3:
        border_colors = [border_colors] * len(images)
    elif len(border_colors) < len(images):
        # Заполняем недостающие цвета последним
        border_colors += [border_colors[-1]] * (len(images) - len(border_colors))

    # Приводим все изображения к 3-канальному виду
    imgs = []
    for img in images:
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        imgs.append(img)

    # Выравниваем размеры
    h_max = max(img.shape[0] for img in imgs)
    w_max = max(img.shape[1] for img in imgs)

    imgs_prepared = []
    for idx, img in enumerate(imgs):
        resized = cv2.resize(img, (w_max, h_max))

        # Цвет рамки для текущего изображения
        color = border_colors[idx]

        # Добавляем рамку
        bordered = cv2.copyMakeBorder(
            resized,
            border_thickness, border_thickness,
            border_thickness, border_thickness,
            cv2.BORDER_CONSTANT,
            value=color
        )

        # Добавляем подпись, если есть
        if captions and idx < len(captions):
            caption = captions[idx]
            canvas = np.full(
                (bordered.shape[0] + caption_height, bordered.shape[1], 3),
                bg_color,
                dtype=np.uint8
            )
            canvas[:bordered.shape[0], :] = bordered
            (text_w, text_h), _ = cv2.getTextSize(caption, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
            x = (bordered.shape[1] - text_w) // 2
            y = bordered.shape[0] + (caption_height + text_h) // 2
            cv2.putText(canvas, caption, (x, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_color, font_thickness, cv2.LINE_AA)
            imgs_prepared.append(canvas)
        else:
            imgs_prepared.append(bordered)

    # Новые размеры (учитываем рамку и подписи)
    h_max = max(img.shape[0] for img in imgs_prepared)
    w_max = max(img.shape[1] for img in imgs_prepared)

    # Объединяем
    if layout == 'horizontal':
        combined = cv2.hconcat(imgs_prepared)

    elif layout == 'vertical':
        combined = cv2.vconcat(imgs_prepared)

    elif layout == 'grid':
        if grid_size is None:
            cols = int(np.ceil(np.sqrt(len(imgs_prepared))))
            rows = int(np.ceil(len(imgs_prepared) / cols))
        else:
            rows, cols = grid_size

        H = rows * h_max + (rows - 1) * spacing
        W = cols * w_max + (cols - 1) * spacing
        combined = np.full((H, W, 3), bg_color, dtype=np.uint8)

        for idx, img in enumerate(imgs_prepared):
            r = idx // cols
            c = idx % cols
            y = r * (h_max + spacing)
            x = c * (w_max + spacing)
            combined[y:y+img.shape[0], x:x+img.shape[1]] = img
    else:
        raise ValueError("layout должен быть 'horizontal', 'vertical' или 'grid'")

    return combined
