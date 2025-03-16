import cv2
import numpy as np

# Load video
video_path = "C:/Users/mahek/mahek/opencv/volleyball_match.mp4"
cap = cv2.VideoCapture(video_path)

# Yellow color range (refined)
yellow_lower = np.array([10, 100, 100])  
yellow_upper = np.array([40, 255, 255])  

def circularity(area, perimeter):
    if perimeter > 0:
        return (4 * np.pi * area) / (perimeter ** 2)
    return 0

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create mask for yellow color
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

    # Morphological operations to clean noise
    kernel = np.ones((5, 5), np.uint8)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
    yellow_mask = cv2.dilate(yellow_mask, kernel, iterations=2)

    # Find contours
    contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    max_score = 0
    best_contour = None

    for cnt in contours:
        area = cv2.contourArea(cnt)
        perimeter = cv2.arcLength(cnt, closed=True)

        if 300 < area < 2000:  # Ensure reasonable size
            score = circularity(area, perimeter)
            if 0.7 < score < 1.2:  # Ensures round shape
                if score > max_score:
                    max_score = score
                    best_contour = cnt

    # Draw detected ball
    if best_contour is not None:
        x, y, w, h = cv2.boundingRect(best_contour)
        center = (x + w // 2, y + h // 2)
        cv2.circle(frame, center, 10, (0, 0, 255), 3)  # Draw red circle

    # Display results
    cv2.imshow("Volleyball Detection", frame)
    cv2.imshow("Yellow Mask", yellow_mask)

    # Exit on 'q' key
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


