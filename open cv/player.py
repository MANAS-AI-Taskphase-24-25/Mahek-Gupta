import numpy as np
import cv2 as cv

#yellow_team_history=[]
#red_team_history=[]

# Load video
video_path = "C:/Users/mahek/mahek/opencv/volleyball_match.mp4"
cap = cv.VideoCapture(video_path)

# HSV Color Ranges
yellow_lower = np.array([10, 100, 100])  
yellow_upper = np.array([40, 255, 255])

blue_lower = np.array([100, 150, 50])  
blue_upper = np.array([140, 255, 255])

red_lower = np.array([174, 137, 100])  
red_upper = np.array([176, 255, 255])

white_lower = np.array([0, 0, 200])  # Adjusted to avoid background
white_upper = np.array([180, 40, 255])  

# Processing Loop
while True:
    ret, frame = cap.read()
    if not ret:
        break

    roi = frame[150:600, 100:1100]  # Region of interest to focus on the court
    hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)

    # Creating masks
    yellow_mask = cv.inRange(hsv, yellow_lower, yellow_upper)
    blue_mask = cv.inRange(hsv, blue_lower, blue_upper)
    red_mask = cv.inRange(hsv, red_lower, red_upper)
    white_mask = cv.inRange(hsv, white_lower, white_upper)

    # Combine yellow and blue for Team 1
    team1_mask = cv.bitwise_or(yellow_mask, blue_mask)

    # Combine red and white for Team 2
    team2_mask = cv.bitwise_or(red_mask, white_mask)

    # Remove noise and small artifacts
    kernel = np.ones((5, 5), np.uint8)
    team1_mask = cv.morphologyEx(team1_mask, cv.MORPH_CLOSE, kernel)
    team1_mask = cv.dilate(team1_mask, kernel, iterations=2)

    team2_mask = cv.morphologyEx(team2_mask, cv.MORPH_CLOSE, kernel)
    team2_mask = cv.dilate(team2_mask, kernel, iterations=2)

    # Find contours for each team
    team1_contours, _ = cv.findContours(team1_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    team2_contours, _ = cv.findContours(team2_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    team1 = 0
    team2 = 0

    def is_valid_player(cnt):
        area = cv.contourArea(cnt)
        x, y, w, h = cv.boundingRect(cnt)
        aspect_ratio = w / float(h)
        return 2000 < area < 5000 and 0.2 < aspect_ratio < 0.9  # Ensures human-like shape

    # Process Team 1 (Yellow + Blue)
    for cnt in team1_contours:
        if is_valid_player(cnt):
            x, y, w, h = cv.boundingRect(cnt)
            cv.rectangle(roi, (x, y), (x + w, y + h), (0, 255, 255), 2)  # Yellow Box
            team1 += 1

    # Process Team 2 (Red + White)
    for cnt in team2_contours:
        if is_valid_player(cnt):
            x, y, w, h = cv.boundingRect(cnt)
            cv.rectangle(roi, (x, y), (x + w, y + h), (255, 0, 255), 2)  # Red Box
            team2 += 1
    #yellow_team_history.append(team1)
    #red_team_history.append(team2)


    # Display Player Count
    cv.putText(roi, f"Team 1: {team1}", (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    cv.putText(roi, f"Team 2: {team2}", (50, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
    

    # Show Output
    cv.imshow("Volleyball Player Detection", roi)

    # Exit on 'q' key
    if cv.waitKey(40) & 0xFF == ord('q'):
        break
# After the loop, calculate the average number of yellow players
#total_yellow = sum(yellow_team_history)
#average_yellow =round( total_yellow / len(yellow_team_history)) if len(yellow_team_history) > 0 else 0

# Calculate the average number of red players
#total_red = sum(red_team_history)
#average_red =round( total_red / len(red_team_history) )if len(red_team_history) > 0 else 0

# Print the final average results
#print(f"Average number of yellow players: {average_yellow}")
#print(f"Average number of red players: {average_red}")

cap.release()
cv.destroyAllWindows()


