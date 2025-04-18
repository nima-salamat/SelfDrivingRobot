import cv2
import numpy as np

def process_image(image):
    # Step 1: Convert image to grayscale and apply binary threshold
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    
    # Step 2: Detect lines using Hough Transform
    edges = cv2.Canny(binary, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=100, minLineLength=100, maxLineGap=10)
    
    # Initialize variables for left and right lines
    left_line_x = []
    right_line_x = []
    y_coords = []
    
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # Calculate slope to determine if line is vertical-ish
            if abs(x2 - x1) > 10:  # Avoid nearly horizontal lines
                continue
            x_avg = (x1 + x2) // 2
            # Group lines by their x-coordinates
            if x_avg < image.shape[1] // 3:
                left_line_x.append(x_avg)
            elif x_avg > 2 * image.shape[1] // 3:
                right_line_x.append(x_avg)
            y_coords.append((y1 + y2) // 2)
    
    # Step 3: Calculate middle point of left and right lines
    left_x = int(np.mean(left_line_x)) if left_line_x else 0
    right_x = int(np.mean(right_line_x)) if right_line_x else image.shape[1]
    
    # Ignore any line close to the middle of left and right lines
    middle_x = (left_x + right_x) // 2
    filtered_lines = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            x_avg = (x1 + x2) // 2
            # Ignore lines within 10 pixels of the middle
            if abs(x_avg - middle_x) > 10:
                filtered_lines.append(line)
    
    # Draw lines and middle point
    output = image.copy()
    if filtered_lines:
        for line in filtered_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
    # Draw middle point
    middle_y = int(np.mean(y_coords)) if y_coords else image.shape[0] // 2
    cv2.circle(output, (middle_x, middle_y), 5, (0, 0, 255), -1)
    
    # Display coordinates
    cv2.putText(output, f"Middle: ({middle_x}, {middle_y})", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    
    return output, (middle_x, middle_y)

def main():
    # Initialize camera
    cap = cv2.VideoCapture(0)  # Use 0 for default camera
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # Process frame
        processed_frame, middle_coords = process_image(frame)
        
        # Display output
        cv2.imshow('Road Detection', processed_frame)
        
        # Print middle coordinates (for robot control)
        print(f"Middle coordinates: {middle_coords}")
        
        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()