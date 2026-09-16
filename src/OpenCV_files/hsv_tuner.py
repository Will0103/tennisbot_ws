import cv2
import numpy as np


# ============================================================
# Webcam 設定
# ============================================================

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

cap.set(
    cv2.CAP_PROP_FOURCC,
    cv2.VideoWriter_fourcc(*"MJPG")
)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)


# ============================================================
# Trackbar callback
# OpenCV 要求 createTrackbar 一定要有 callback
# 但我們不需要它做任何事情
# ============================================================

def nothing(x):
    pass


# ============================================================
# HSV → BGR
#
# 因為 OpenCV 顯示圖片是 BGR，
# 所以要把我們選的 HSV 顏色轉成 BGR 才能畫出來
# ============================================================

def hsv_to_bgr(h, s, v):

    hsv_pixel = np.uint8([[[h, s, v]]])

    bgr_pixel = cv2.cvtColor(
        hsv_pixel,
        cv2.COLOR_HSV2BGR
    )

    b = int(bgr_pixel[0][0][0])
    g = int(bgr_pixel[0][0][1])
    r = int(bgr_pixel[0][0][2])

    return (b, g, r)


# ============================================================
# 建立 Hue 顏色範圍條
#
# 例如：
#
# H Min = 25
# H Max = 80
#
# 就會把 25 ~ 80 對應的顏色畫出來
# ============================================================

def create_hue_bar(h_min, h_max, width=640, height=30):

    bar = np.zeros(
        (height, width, 3),
        dtype=np.uint8
    )

    for x in range(width):

        ratio = x / (width - 1)

        hue = int(
            h_min +
            ratio * (h_max - h_min)
        )

        color = hsv_to_bgr(
            hue,
            255,
            255
        )

        bar[:, x] = color

    return bar


# ============================================================
# 建立 HSV Tuner 視窗
# ============================================================

cv2.namedWindow(
    "HSV Tuner",
    cv2.WINDOW_NORMAL
)


# ============================================================
# 建立 Trackbar
# ============================================================

# Hue 範圍：0 ~ 179
cv2.createTrackbar(
    "H Min",
    "HSV Tuner",
    27,
    179,
    nothing
)

cv2.createTrackbar(
    "H Max",
    "HSV Tuner",
    52,
    179,
    nothing
)


# Saturation 範圍：0 ~ 255
cv2.createTrackbar(
    "S Min",
    "HSV Tuner",
    65,
    255,
    nothing
)

cv2.createTrackbar(
    "S Max",
    "HSV Tuner",
    255,
    255,
    nothing
)


# Value 範圍：0 ~ 255
cv2.createTrackbar(
    "V Min",
    "HSV Tuner",
    86,
    255,
    nothing
)

cv2.createTrackbar(
    "V Max",
    "HSV Tuner",
    255,
    255,
    nothing
)


# ============================================================
# Main Loop
# ============================================================

while True:

    # --------------------------------------------------------
    # 讀取 Webcam
    # --------------------------------------------------------

    ret, frame = cap.read()

    if not ret:
        print("Failed to read camera")
        break


    # --------------------------------------------------------
    # BGR → HSV
    # --------------------------------------------------------

    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV
    )


    # --------------------------------------------------------
    # 讀取 Trackbar 數值
    # --------------------------------------------------------

    h_min = cv2.getTrackbarPos(
        "H Min",
        "HSV Tuner"
    )

    h_max = cv2.getTrackbarPos(
        "H Max",
        "HSV Tuner"
    )

    s_min = cv2.getTrackbarPos(
        "S Min",
        "HSV Tuner"
    )

    s_max = cv2.getTrackbarPos(
        "S Max",
        "HSV Tuner"
    )

    v_min = cv2.getTrackbarPos(
        "V Min",
        "HSV Tuner"
    )

    v_max = cv2.getTrackbarPos(
        "V Max",
        "HSV Tuner"
    )


    # --------------------------------------------------------
    # 建立 HSV Lower / Upper
    # --------------------------------------------------------

    lower = np.array(
        [h_min, s_min, v_min]
    )

    upper = np.array(
        [h_max, s_max, v_max]
    )


    # --------------------------------------------------------
    # 建立 Mask
    #
    # HSV 在範圍內 → 白色
    # HSV 不在範圍內 → 黑色
    # --------------------------------------------------------

    mask = cv2.inRange(
        hsv,
        lower,
        upper
    )


    # --------------------------------------------------------
    # 把 Mask 套回原本影像
    #
    # 只留下符合 HSV 範圍的顏色
    # --------------------------------------------------------

    result = cv2.bitwise_and(
        frame,
        frame,
        mask=mask
    )


    # ========================================================
    # 建立資訊區域
    # ========================================================

    info = np.zeros(
        (110, 640, 3),
        dtype=np.uint8
    )


    # --------------------------------------------------------
    # HSV 數值文字
    # --------------------------------------------------------

    lower_text = (
        f"Lower HSV : "
        f"H={h_min}  "
        f"S={s_min}  "
        f"V={v_min}"
    )

    upper_text = (
        f"Upper HSV : "
        f"H={h_max}  "
        f"S={s_max}  "
        f"V={v_max}"
    )


    cv2.putText(
        info,
        lower_text,
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2
    )

    cv2.putText(
        info,
        upper_text,
        (10, 55),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2
    )


    # ========================================================
    # Lower / Upper HSV 對應顏色
    # ========================================================

    lower_color = hsv_to_bgr(
        h_min,
        s_min,
        v_min
    )

    upper_color = hsv_to_bgr(
        h_max,
        s_max,
        v_max
    )


    # Lower 色塊
    cv2.rectangle(
        info,
        (470, 5),
        (540, 35),
        lower_color,
        -1
    )


    # Upper 色塊
    cv2.rectangle(
        info,
        (550, 5),
        (620, 35),
        upper_color,
        -1
    )


    cv2.putText(
        info,
        "LOW",
        (480, 55),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1
    )

    cv2.putText(
        info,
        "HIGH",
        (555, 55),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1
    )


    # ========================================================
    # Hue 範圍條
    # ========================================================

    hue_bar = create_hue_bar(
        h_min,
        h_max,
        640,
        30
    )


    # ========================================================
    # 合併成 HSV Tuner 畫面
    #
    # result
    # ↓
    # info
    # ↓
    # hue bar
    # ========================================================

    tuner_display = np.vstack(
        (
            result,
            info,
            hue_bar
        )
    )


    # ========================================================
    # 顯示
    # ========================================================

    cv2.imshow(
        "HSV Tuner",
        tuner_display
    )

    # cv2.imshow(
    #     "Mask",
    #     mask
    # )

    # cv2.imshow(
    #     "Webcam",
    #     frame
    # )


    # ========================================================
    # 按 q 離開
    # ========================================================

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):

        print("\nFinal HSV Range:")

        print(
            f"lower = np.array("
            f"[{h_min}, {s_min}, {v_min}])"
        )

        print(
            f"upper = np.array("
            f"[{h_max}, {s_max}, {v_max}])"
        )

        break


# ============================================================
# 關閉 Camera / Window
# ============================================================

cap.release()

cv2.destroyAllWindows()