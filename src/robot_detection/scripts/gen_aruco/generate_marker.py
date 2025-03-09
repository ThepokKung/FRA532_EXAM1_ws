import cv2
import cv2.aruco as aruco

# ตั้งค่า
NUM_MARKERS = 9
MARKER_SIZE = 500  # พิกเซล
DICTIONARY = aruco.DICT_4X4_250  # ต้องตรงกับโค้ดตรวจจับ

for i in range (NUM_MARKERS):
    # สร้างภาพ marker
    aruco_dict = aruco.getPredefinedDictionary(DICTIONARY)
    marker_img = aruco.generateImageMarker(aruco_dict, i, MARKER_SIZE)
    
    # บันทึกไฟล์
    filename = f"aruco_marker_{i}.png"
    cv2.imwrite(filename, marker_img)
    print(f"Saved {filename}")