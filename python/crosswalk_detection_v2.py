import cv2


class CrosswalkDetector:
    def detect(self, frame, roi):
        # Extract the ROI from the frame
        roi_frame = frame[roi[0][0] : roi[0][1], roi[1][0] : roi[1][1]]

        # Convert to grayscale
        gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian Blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Use a lower threshold to capture less bright lines (adjustable)
        _, thresh = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY)

        # Define a horizontal kernel (adjust size based on resolution)
        horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (40, 1))
        detected_lines = cv2.morphologyEx(
            thresh, cv2.MORPH_OPEN, horizontal_kernel, iterations=2
        )

        # Find contours
        contours, _ = cv2.findContours(
            detected_lines, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # Flag to indicate detection
        crosswalk_detected = False

        # Process each contour
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            # Relax height constraint since crosswalk lines may be thin
            if w > 40 and h > 10:
                # Draw a green rectangle on the ROI (this modifies the original frame)
                cv2.rectangle(roi_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                print(f"Detected contour: width={w}, height={h}")
                crosswalk_detected = True

        return crosswalk_detected
