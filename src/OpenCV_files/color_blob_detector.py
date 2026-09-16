import cv2
import numpy as np

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

# Color 1
lower1 = np.array([31, 65, 86])
upper1 = np.array([52, 255, 255])
# Color 2
lower2 = np.array([3, 98, 104])
upper2 = np.array([15, 255, 255])

kernel = np.ones((11, 11), np.uint8)
while True:
    ret, frame = cap.read()

    if not ret:
        print("Failed to read camera")
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv, lower1, upper1)
    mask2 = cv2.inRange(hsv, lower2, upper2)
    mask = cv2.bitwise_or(mask1, mask2)

    #Close
    mask = cv2.morphologyEx(
    mask,
    cv2.MORPH_CLOSE,
    kernel,
    iterations=1
    )

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )
    contour_image = frame.copy()

    cv2.drawContours(
        contour_image,
        contours,
        -1,
        (0, 255, 0),
        2
    )

    cv2.imshow("Contour", contour_image)


    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        perimeter = cv2.arcLength(largest_contour, True)

        if perimeter > 0:
            circularity = 4 * np.pi * area / (perimeter * perimeter)
        else:
            circularity = 0
        
        if area > 300 and circularity > 0.60:
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)

            center = (int(x), int(y))
            radius = int(radius)
            diameter_pixel = radius * 2

            cv2.circle(frame, center, radius, (0, 255, 0), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            cv2.putText(
                frame,
                f"Center: {center}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7, #font size
                (0, 255, 0),
                2    #line size
            )

            cv2.putText(
                frame,
                f"Diameter: {diameter_pixel} px",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                f"Circularity: {circularity:.2f} ",
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

    cv2.imshow("Webcam", frame)
    # cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
