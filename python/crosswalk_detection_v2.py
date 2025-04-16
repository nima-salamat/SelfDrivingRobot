import cv2



class CrosswalkDetector:
   
    def detect(self, frame, roi):
        frame = frame[roi[0][0]:roi[0][1],roi[1][0]:roi[1][1]]
         # تبدیل به Grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # اعمال Gaussian Blur برای کاهش نویز
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Threshold برای تشخیص نواحی سفید
        _, thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)

        # ایجاد kernel افقی برای تشخیص خطوط افقی
        horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 1))
        detected_lines = cv2.morphologyEx(
            thresh, cv2.MORPH_OPEN, horizontal_kernel, iterations=2
        )

        # یافتن کانتورها
        contours, _ = cv2.findContours(
            detected_lines, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # رسم خطوط سفید ضخیم
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if w > 50 and h > 15:  # فیلتر بر اساس اندازه
                return True
        return False