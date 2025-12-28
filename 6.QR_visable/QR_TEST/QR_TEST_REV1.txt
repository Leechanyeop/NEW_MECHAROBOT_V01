import cv2
import numpy as np
import time

class CameraContourDetector:
    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open the camera")

        # QR 코드 디텍터 추가
        self.qr_detector = cv2.QRCodeDetector()

        cv2.namedWindow("Original Video", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Canny Edges", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Contour Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Original Video", 640, 480)
        cv2.resizeWindow("Canny Edges", 640, 480)
        cv2.resizeWindow("Contour Detection", 640, 480)

    def preprocess_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        return blurred

    def detect_edges(self, preprocessed_frame):
        edges = cv2.Canny(preprocessed_frame, 30, 150)
        dilated = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=2)
        return dilated

    def find_and_draw_contours(self, frame, edges):
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour_image = frame.copy()
        cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 2)
        contour_count = len(contours)
        cv2.putText(contour_image, f"Contour Count: {contour_count}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        return contour_image, contour_count

    def detect_qr(self, frame):
        """QR 코드 인식"""
        data, bbox, _ = self.qr_detector.detectAndDecode(frame)
        if data:
            cv2.putText(frame, f"QR: {data}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        return frame, data

    def run(self):
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to get the image frame")
                    break

                frame = cv2.flip(frame, 1)

                preprocessed = self.preprocess_frame(frame)
                edges = self.detect_edges(preprocessed)
                contour_image, count = self.find_and_draw_contours(frame, edges)

                # QR 코드 인식 추가
                qr_frame, qr_data = self.detect_qr(contour_image)
                if qr_data:
                    print("QR 코드 값:", qr_data)

                cv2.imshow("Original Video", frame)
                cv2.imshow("Canny Edges", edges)
                cv2.imshow("Contour Detection", qr_frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
        finally:
            self.cap.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = CameraContourDetector(camera_index=1)  # 필요시 인덱스 변경
    detector.run()