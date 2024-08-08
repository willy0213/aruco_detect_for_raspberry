import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2 as cv
from cv2 import aruco
import numpy as np
import math

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用變量警告
        self.br = CvBridge()  # 初始化 CvBridge 用於轉換 ROS 影像訊息

        # 其他參數設置
        self.id_to_find_16 = 16
        self.id_to_find_0 = 0
        self.marker_size_16 = 500  # 單位公分
        self.marker_size_0 = 100   # 單位公分

    # 檢查矩陣是否為有效的旋轉矩陣
    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    # 計算旋轉矩陣到歐拉角
    def rotationMatrixToEulerAngles(self, R):
        assert (self.isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def listener_callback(self, msg):
        # 設置字體
        font = cv2.FONT_HERSHEY_SIMPLEX

        # 使用 CvBridge 將 ROS 影像訊息轉換為 OpenCV 格式
        current_frame = self.br.imgmsg_to_cv2(msg)

        # 設置內部參數
        self.camera_matrix = np.array([[530.8269276712998, 0.0, 320.5],
                                       [0.0, 530.8269276712998, 240.5],
                                       [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeff = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

        # 獲取預定義的 ArUco 字典
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)

        # 設定 ArUco 檢測參數
        parameters = aruco.DetectorParameters()

        # 將畫面轉成灰階
        gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

        # 找到圖像中的所有aruco標記
        corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)

        if ids is not None:
            for i, marker_id in enumerate(ids):
                # 確認 ArUco 標記的類型和大小
                if marker_id == self.id_to_find_16:
                    marker_size = self.marker_size_16  # 設置大標記的大小
                elif marker_id == self.id_to_find_0:
                    marker_size = self.marker_size_0   # 設置小標記的大小
                else:
                    continue  # 如果不是指定的標記，則跳過處理

                # 估算 ArUco 標記的姿態和位置
                rVec, tVec, _ = aruco.estimatePoseSingleMarkers(corners[i], marker_size, self.camera_matrix, self.dist_coeff)

                R, _ = cv2.Rodrigues(rVec)  # 将旋转向量（rvec）转换为旋转矩阵（R）

                # 計算鏡頭的 -Y 轴与 ArUco 标记的 +Y 轴之间的角度差
                camera_y_axis = -R[:, 1]  # 鏡頭的 -Y 轴
                aruco_y_axis = np.array([0, 1, 0])  # ArUco 标记的 +Y 轴

                # 计算两个向量之间的夹角
                dot_product = np.dot(camera_y_axis, aruco_y_axis)
                norms_product = np.linalg.norm(camera_y_axis) * np.linalg.norm(aruco_y_axis)
                angle_y_radians = np.arccos(dot_product / norms_product)
                angle_y_degrees = np.degrees(angle_y_radians)

                # 调整角度范围到 0-360 度
                if camera_y_axis[0] < 0:
                    angle_y_degrees = 360 - angle_y_degrees

                cv2.putText(current_frame, "Y axis angle: " + str(round(angle_y_degrees, 2)) + " degrees", (0, 150), font, 1, (0, 0, 255), 2, cv2.LINE_AA)

                # 提取標記的角點位置
                corner_points = corners[i].reshape(4, 2)
                corner_points = corner_points.astype(int)

                top_right = corner_points[0].ravel()
                bottom_right = corner_points[2].ravel()

                # 計算飛機相對於標記的位置並顯示
                pos_drone = np.array([tVec[0][0][0], tVec[0][0][1], tVec[0][0][2]])  # 轉換為飛機座標

                # 在標記上方顯示標記的 ID 和距離
                cv2.putText(current_frame, f"id: {marker_id[0]}", top_right, cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255), 1, cv2.LINE_AA)

                # 在標記右下方顯示飛機相對於標記的坐標位置
                cv2.putText(current_frame, f"x:{round(pos_drone[0], 1)} y:{round(pos_drone[1], 1)} z:{round(pos_drone[2], 1)}", bottom_right, cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255), 1, cv2.LINE_AA)

        # 顯示處理過的影像
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
