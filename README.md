🟦 1. Depth Image

What it is:
A map of distances between the camera and every visible point in the scene.

Each pixel value represents how far that point is from the camera — usually in millimeters.
For example:

A pixel value of 500 means the object is 500 mm (50 cm) away.

A pixel value of 2000 means 2 m away.

Use:
This is the main data you use for 3D perception, object detection, distance measurement, or camera tracking.

Visualization:
You can display it in grayscale or a color map (e.g., “rainbow”) where closer objects are brighter or colored differently.

🟩 2. Confidence Image

What it is:
A measure of how reliable each depth measurement is.
Each pixel tells you how confident the camera is that its depth value is correct.

High confidence → reliable depth value (good signal quality).

Low confidence → unreliable measurement (e.g., reflections, low light, or weak return signal).

Range:
Usually 0–255 or 0–1 (depending on SDK output format).

Use:
You typically filter out low-confidence pixels before processing the depth map, to reduce noise and errors.

🟧 3. Amplitude Image (sometimes called Intensity)

What it is:
The strength of the returned light signal — how much infrared light reflected from the object reached the sensor.

Higher amplitude → object reflects a lot of IR light (bright, close, or good surface).

Lower amplitude → weak reflection (dark, far, or absorbing surface).

Use:

To check if an area is too dark or reflective for good depth readings.

To assist confidence filtering (low amplitude usually means low confidence).

🧠 In summary:
Image           Type	                                Meaning	Typical Use
Depth	          Distance to each pixel (mm)	          Measure distance, 3D vision, tracking
Confidence	    Reliability of each depth pixel	      Filter out bad measurements
Amplitude	      Strength of reflected IR signal	      Detect low-light or poor-reflectivity regions
