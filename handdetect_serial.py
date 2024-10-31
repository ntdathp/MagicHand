import cv2
import mediapipe as mp
import serial
import time

# Khởi tạo MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils

# Khởi tạo video từ camera
cap = cv2.VideoCapture(0)

# Khởi tạo cổng COM (cấu hình tùy thuộc vào thông số cổng của bạn)
ser = serial.Serial(
            port="/dev/ttyUSB0",
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
time.sleep(2)  # Chờ cổng COM khởi động

def check_finger_status(landmarks):
    finger_status = []
    
    # Các điểm đặc trưng cho từng ngón tay
    tips_ids = [4, 8, 12, 16, 20]  # ID của các điểm đầu ngón tay: ngón cái, trỏ, giữa, áp út, út

    # Kiểm tra ngón cái với trạng thái đảo ngược (Up thành Down và ngược lại)
    if landmarks[tips_ids[0]].x < landmarks[tips_ids[0] - 1].x:
        finger_status.append("0")  # "Down" là 0
    else:
        finger_status.append("1")  # "Up" là 1

    # Kiểm tra các ngón còn lại (so sánh với khớp giữa của từng ngón)
    for tip_id in tips_ids[1:]:
        if landmarks[tip_id].y < landmarks[tip_id - 2].y:
            finger_status.append("1")  # "Up" là 1
        else:
            finger_status.append("0")  # "Down" là 0

    return finger_status

# Thời gian cuối cùng đã gửi dữ liệu qua cổng COM
last_sent_time = time.time()

while cap.isOpened():
    success, image = cap.read()
    if not success:
        print("Không thể đọc khung hình từ camera.")
        break

    # Chuyển đổi hình ảnh sang RGB để xử lý bằng MediaPipe
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image.flags.writeable = False
    results = hands.process(image)

    # Chuyển đổi hình ảnh về lại BGR để hiển thị bằng OpenCV
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    # Kiểm tra xem có phát hiện bàn tay nào không
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Vẽ các điểm và liên kết trên bàn tay
            mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Kiểm tra trạng thái từng ngón tay
            finger_status = check_finger_status(hand_landmarks.landmark)
            fingers = ["Thumb", "Index Finger", "Middle Finger", "Ring Finger", "Pinky"]

            for finger, status in zip(fingers, finger_status):
                display_text = "Up" if status == "1" else "Down"
                cv2.putText(image, f"{finger}: {display_text}", (10, 40 + fingers.index(finger) * 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Gửi dữ liệu qua cổng COM mỗi 1 giây
            current_time = time.time()
            if current_time - last_sent_time >= 1:  # Kiểm tra nếu đã qua 1 giây
                # Tạo chuỗi trạng thái các ngón tay
                finger_status_str = ','.join(finger_status)  # Chuyển đổi thành dãy số dạng "1,1,0,0,1"
                ser.write(finger_status_str.encode())  # Gửi chuỗi qua cổng COM
                print(finger_status_str)
                last_sent_time = current_time  # Cập nhật thời gian gửi

    # Hiển thị hình ảnh kết quả
    cv2.imshow("Finger Detection", image)

    # Nhấn 'q' để thoát
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Giải phóng bộ nhớ và đóng cổng COM
cap.release()
cv2.destroyAllWindows()
hands.close()
ser.close()
