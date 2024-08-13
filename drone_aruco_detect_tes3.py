import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2 as cv
from cv2 import aruco
import numpy as np
import pymavlink.mavutil as utility
from pymavlink import mavutil
import pymavlink.dialects.v20.all as dialect
import math, sys, time

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(Image,'/camera/image',self.listener_callback,10)
        self.subscription  # 防止未使用變量警告
        self.br = CvBridge()  # 初始化 CvBridge 用於轉換 ROS 影像訊息

        # 連接到飛機
        self.vehicle = utility.mavlink_connection('udpin:127.0.0.1:14550')
        self.vehicle.wait_heartbeat()
        print("Connected to system:", self.vehicle.target_system, ", component:", self.vehicle.target_component)

        # 清除原有任務
        self.clear_mission()

        # 更改 Auto 模式
        self.mode_change_to_Auto()

        # 上傳任務點
        self.set_mission_points()

        # 確認任務總數
        self.get_waypoint_count()

        # 設置 WP_YAW_BEHAVIOR 為 1 （跟著航向）
        self.set_wp_yaw_behavior(1)
        print("WP_YAW_BEHAVIOR set to 1")

        # 其他參數設置
        self.id_to_find_16 = 16
        self.id_to_find_0 = 0
        self.marker_size_16 = 500  # 單位公分
        self.marker_size_0 = 100    # 單位公分

        #解鎖
        self.arm()
    
    def listener_callback(self, msg):

        # 得到當前任務點
        self.get_waypoint_info()

        # 檢查是否到達任務點
        if self.wp_point == self.wp_point_count:

            #  檢查當前模式是否為 GUIDED
            if self.flight_mode_name ==  "GUIDED":

                #print("飛行模式正確，開始進行 ArUco 定位")

                #-- 設置字體
                font = cv2.FONT_HERSHEY_SIMPLEX

                # 日誌訊息，提示正在接收影像幀
                #self.get_logger().info('Receiving video frame')

                # 使用 CvBridge 將 ROS 影像訊息轉換為 OpenCV 格式
                current_frame = self.br.imgmsg_to_cv2(msg)

                # 設置相機內部參數
                self.camera_matrix = np.array([[530.8269276712998, 0.0, 320.5],
                                            [0.0, 530.8269276712998, 240.5],
                                            [0.0, 0.0, 1.0]], dtype=np.float32)
                self.dist_coeff = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
                
                # 獲取預定義的 ArUco 字典
                aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)

                # 設定 ArUco 檢測參數
                parameters = aruco.DetectorParameters()
                parameters.adaptiveThreshWinSizeMin = 3
                parameters.adaptiveThreshWinSizeMax = 23
                parameters.adaptiveThreshWinSizeStep = 10
                parameters.minMarkerPerimeterRate = 0.03
                parameters.maxMarkerPerimeterRate = 4.0

                # 將畫面轉成灰階
                gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

                #找到圖像中的所有aruco標記
                corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)

                if ids is not None:
                    for i, self.id in enumerate(ids):

                        # 確認 ArUco 標記的類型和大小
                        if self.id == self.id_to_find_16:
                            marker_size = self.marker_size_16  # 設置大標記的大小
                        elif self.id == self.id_to_find_0:
                            marker_size = self.marker_size_0   # 設置小標記的大小
                        else:
                            continue  # 如果不是指定的標記，則跳過處理

                        # 估算 ArUco 標記的姿態和位置
                        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(corners[i], marker_size, self.camera_matrix, self.dist_coeff)

                        R, _ = cv2.Rodrigues(rVec)  # 將旋轉向量（rVec）轉換為旋轉矩陣（R）

                        # 計算鏡頭的 -Y 軸與 ArUco 的 +Y 軸之間的角度差
                        camera_y_axis = -R[:, 1]  # 鏡頭的 -Y 軸
                        aruco_y_axis = np.array([0, 1, 0])  # ArUco 的 +Y 軸

                        # 計算兩個向量之間的夾角
                        dot_product = np.dot(camera_y_axis, aruco_y_axis)
                        norms_product = np.linalg.norm(camera_y_axis) * np.linalg.norm(aruco_y_axis)
                        angle_y_radians = np.arccos(dot_product / norms_product)
                        angle_y_degrees = np.degrees(angle_y_radians)

                        # 調整角度範圍到 0-360 度
                        if camera_y_axis[0] < 0:
                            angle_y_degrees = 360 - angle_y_degrees
                        
                        # 得到角度差
                        self.aruco_yaw = (round(angle_y_degrees, 2))

                        # 獲得標記的角點位置
                        corner_points = corners[i].reshape(4, 2)
                        corner_points = corner_points.astype(int)

                        top_right = corner_points[0].ravel()
                        top_left = corner_points[1].ravel()
                        bottom_right = corner_points[2].ravel()
                        bottom_left = corner_points[3].ravel()

                        # 從飛機獲取當前高度數據
                        altitude_data = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                        self.altitude = altitude_data.relative_alt / 1000.0 if altitude_data else 0

                        # 繪製標記的座標軸
                        cv2.drawFrameAxes(current_frame, self.camera_matrix, self.dist_coeff, rVec[0], tVec[0], 4)

                        # 計算飛機相對於標記的位置並顯示
                        pos_drone = np.array([tVec[0][0][1], -tVec[0][0][0], tVec[0][0][2]])  # 轉換為飛機座標
                        #str_position_drone = f"Drone Position (relative to Marker {ids[0]}) x={pos_drone[0]:.2f} y={pos_drone[1]:.2f} z={pos_drone[2]:.2f}"
                        #cv2.putText(current_frame, str_position_drone, (0, 100 + i * 100), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 255, 0), 1, cv2.LINE_AA)

                        # 在標記上方顯示標記的 ID 和距離
                        cv2.putText(current_frame, f"id: {ids[0]} Alt: {round(self.altitude, 2)}m", top_right, cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255), 1, cv2.LINE_AA)

                        # 在標記右下方顯示飛機相對於標記的坐標位置
                        cv2.putText(current_frame, f"x:{round(pos_drone[0], 1)} y: {round(pos_drone[1], 1)}", bottom_right, cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255), 1, cv2.LINE_AA)

                        # 將 x y 座標存到其他值
                        self.x = pos_drone[0]
                        self.y = pos_drone[1]

                        print("x:", {round(pos_drone[0], 1)}, "y:", {round(pos_drone[1], 1)}, "z:", self.altitude)

                        if 0 > self.aruco_yaw > 15 or 360 > self.aruco_yaw > 345:
                            # 設置 WP_YAW_BEHAVIOR 為 0 （固定航向）
                            self.set_wp_yaw_behavior(0)
                            print("WP_YAW_BEHAVIOR set to 0")

                            # 呼叫精準降落函數
                            self.precise_landing()
                        else:
                            print("與 aruco 角度差:", self.aruco_yaw)
                            self.set_yaw()

                elif ids is None:
                    altitude_data = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                    self.altitude = altitude_data.relative_alt / 1000.0 if altitude_data else 0
                    self.not_found_aruco()

                # 顯示處理過的影像
                #cv2.imshow("camera", current_frame)
                cv2.waitKey(1)

            else:
                # 切換guided模式
                self.mode_change_to_guided()
                time.sleep(2)
        else:
            print("還沒達到最後任務點")
            pass
        
    # 定義任務點函數
    def set_mission_points(self):

        # 得到 home point 位置
        home_position = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

        # 獲得 home point 經度、緯度、高度
        latitude_of_home = home_position.lat / 1e7
        longitude_of_home = home_position.lon / 1e7

        # 創建任務項目列表
        target_locations = ((-35.36305463, 149.16472173, 10.0),
                            (-35.36321446, 149.16529466, 10.0),
                            (-35.36321446, 149.16529466, 10.0))      #latitude_of_home, longitude_of_home, 10.0)

        # 創建任務計數消息
        message = dialect.MAVLink_mission_count_message(target_system=self.vehicle.target_system,
                                                        target_component=self.vehicle.target_component,
                                                        count=len(target_locations) + 2,
                                                        mission_type=dialect.MAV_MISSION_TYPE_MISSION)

        # 發送任務計數消息給飛機
        self.vehicle.mav.send(message)

        # 這個循環會持續到接收到有效的MISSION_ACK消息
        while True:

            # 接收一條消息
            message = self.vehicle.recv_match(blocking=True)

            # 將消息轉換為字典
            message = message.to_dict()

            # 檢查這條消息是MISSION_REQUEST
            if message["mavpackettype"] == dialect.MAVLink_mission_request_message.msgname:
                print(message)

                # 檢查這個請求是任務項目
                if message["mission_type"] == dialect.MAV_MISSION_TYPE_MISSION:

                    # 獲取請求的任務項目序列號
                    seq = message["seq"]

                    # 創建任務項目整數消息
                    if seq == 0:
                        # 創建包含家庭位置的任務項目整數消息（第0個任務項目）
                        message = dialect.MAVLink_mission_item_int_message(target_system=self.vehicle.target_system,
                                                                        target_component=self.vehicle.target_component,
                                                                        seq=seq,
                                                                        frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                                        command=dialect.MAV_CMD_NAV_WAYPOINT,
                                                                        current=0,
                                                                        autocontinue=0,
                                                                        param1=0,
                                                                        param2=0,
                                                                        param3=0,
                                                                        param4=0,
                                                                        x=0,
                                                                        y=0,
                                                                        z=0,
                                                                        mission_type=dialect.MAV_MISSION_TYPE_MISSION)
                    elif seq == 1:
                        # 創建包含起飛命令的任務項目整數消息（第1個任務項目）
                        message = dialect.MAVLink_mission_item_int_message(target_system=self.vehicle.target_system,
                                                                        target_component=self.vehicle.target_component,
                                                                        seq=seq,
                                                                        frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                                        command=dialect.MAV_CMD_NAV_TAKEOFF,
                                                                        current=0,
                                                                        autocontinue=0,
                                                                        param1=0,
                                                                        param2=0,
                                                                        param3=0,
                                                                        param4=0,
                                                                        x=0,
                                                                        y=0,
                                                                        z=target_locations[0][2],
                                                                        mission_type=dialect.MAV_MISSION_TYPE_MISSION)
                    else:
                        # 創建包含目標位置的任務項目整數消息（其他任務項目）
                        message = dialect.MAVLink_mission_item_int_message(target_system=self.vehicle.target_system,
                                                                        target_component=self.vehicle.target_component,
                                                                        seq=seq,
                                                                        frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                                        command=dialect.MAV_CMD_NAV_WAYPOINT,
                                                                        current=0,
                                                                        autocontinue=0,
                                                                        param1=0,
                                                                        param2=0,
                                                                        param3=0,
                                                                        param4=0,
                                                                        x=int(target_locations[seq-2][0] * 1e7),
                                                                        y=int(target_locations[seq-2][1] * 1e7),
                                                                        z=target_locations[seq-2][2],
                                                                        mission_type=dialect.MAV_MISSION_TYPE_MISSION)

                    # 發送任務項目整數消息給飛機
                    self.vehicle.mav.send(message)

            # 檢查這條消息是MISSION_ACK
            elif message["mavpackettype"] == dialect.MAVLink_mission_ack_message.msgname:
                
                # 檢查這個確認消息是任務且已接受
                if message["type"] == dialect.MAV_MISSION_ACCEPTED and message["mission_type"] == dialect.MAV_MISSION_TYPE_MISSION:

                    # 打印上傳任務成功
                    print("Mission upload is successful")
                    break

    # 定義解鎖函數
    def arm(self):

        # 定義解鎖變數
        VEHICLE_ARM = 1

        # 無人機解鎖訊息
        vehicle_arm_message = dialect.MAVLink_command_long_message(
            target_system = self.vehicle.target_system,
            target_component = self.vehicle.target_component,
            command=dialect.MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation=0,
            param1=VEHICLE_ARM,
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )

        # 確認自檢狀態
        while True:

            # 觀察 SYS_STATUS 訊息
            message = self.vehicle.recv_match(type = dialect.MAVLink_sys_status_message.msgname, blocking = True)

            # 轉換為字典
            message = message.to_dict()

            # 取得感測器健康狀況
            onboard_control_sensors_health = message['onboard_control_sensors_health']

            # 解鎖前保持健康
            prearm_status = onboard_control_sensors_health & dialect.MAV_SYS_STATUS_PREARM_CHECK == dialect.MAV_SYS_STATUS_PREARM_CHECK

            if  prearm_status:

                # vehicle can be armable
                print("vehicle is armable")

                # break the prearm check loop
                break
        
        while True:
            # 解鎖飛機
            print("vehicle is arming....")  

            # 將解鎖訊息發送給飛機
            self.vehicle.mav.send(vehicle_arm_message)

            # 等待 COMMAND_ACK MESSAGE
            message = self.vehicle.recv_match(type=dialect.MAVLink_command_ack_message.msgname, blocking=True)

            # 轉換為字典
            message = message.to_dict()

            # 確認飛機是否已成功解鎖
            if message['result'] == dialect.MAV_RESULT_ACCEPTED:
                print("Vehicle is armed!")
                break  # 跳出循环
            else:
                print("Vehicle is not armed!")
                time.sleep(2)  # 等待1秒後再次發送ARM消息
                continue

    # 定義自動模式函數
    def mode_change_to_Auto(self):

        # 定義自動模式變數
        FLIGHT_MODE = 'AUTO'

       # 獲取有支援的模式列表
        flight_modes = self.vehicle.mode_mapping()
    
        # 檢查是否支援所需的飛行模式
        if FLIGHT_MODE not in flight_modes.keys():

            # 通知使用者無人機不支援所需模式
            print(FLIGHT_MODE, 'is not supported')

            # 中斷
            exit()
        
        # 建立更改模式訊息
        set_mode_message = dialect.MAVLink_command_long_message(
            target_system=self.vehicle.target_system,
            target_component=self.vehicle.target_component,
            command=dialect.MAV_CMD_DO_SET_MODE,
            confirmation=0,
            param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            param2=flight_modes[FLIGHT_MODE],
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        # 捕捉心跳訊息
        message = self.vehicle.recv_match(type=dialect.MAVLink_heartbeat_message.msgname, blocking=True)

        # 將此訊息轉換為字典
        message = message.to_dict()

        # 取得模式ID
        mode_id = message['custom_mode']

        # 取得模式名稱
        flight_mode_names = list(flight_modes.keys())
        flight_mode_ids = list(flight_modes.values())
        flight_mode_index = flight_mode_ids.index(mode_id)
        self.flight_mode_name = flight_mode_names[flight_mode_index ]

        #列印心跳訊息
        print("Mode name before:", self.flight_mode_name)

        # 改變飛航模式
        self.vehicle.mav.send(set_mode_message)

        # 始終執行以下操作
        while True:

            # 捕獲 COMMAND_ACK 訊息
            message = self.vehicle.recv_match(type=dialect.MAVLink_command_ack_message.msgname, blocking=True)

            # 將此訊息轉換為字典
            message = message.to_dict()

            # 檢查 COMMAND_ACK 是否適用於 DO_SET_MODE
            if message['command'] == dialect.MAV_CMD_DO_SET_MODE:

                # 檢查命令是否被接受
                if message['result'] == dialect.MAV_RESULT_ACCEPTED:

                    print("Changeing mode to", FLIGHT_MODE, "accepted from the vehicle")

                # 不接受
                else:

                    print("Changeing mode to", FLIGHT_MODE, "failed")

                break

        # 捕捉心跳訊息
        message = self.vehicle.recv_match(type=dialect.MAVLink_heartbeat_message.msgname, blocking=True)

        # 將此訊息轉換為字典
        message = message.to_dict()

        # 取得模式ID
        mode_id = message['custom_mode']

        # 取得模式名稱
        flight_mode_names = list(flight_modes.keys())
        flight_mode_ids = list(flight_modes.values())
        flight_mode_index = flight_mode_ids.index(mode_id)
        self.flight_mode_name = flight_mode_names[flight_mode_index ]

        # 列印心跳訊息
        print("Mode name after:", self.flight_mode_name)

        # 改變飛航模式
        self.vehicle.mav.send(set_mode_message)

    # 定義精確降落函數
    def precise_landing(self):

        # 確認 ArUco 標記的類型
        if self.id == self.id_to_find_16:
            print("Find aruco marker 16")

            # 確認 ArUco 距離飛機距離
            if self.x > 100 and self.y > 100:
                print("ArUco 在 x 正方向且 y 正方向")
                self.vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,self.vehicle.target_system,
                        self.vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111111000) ,-1,-1,0,0,0,0,0,0,0,0,0))
                time.sleep(3)
            
            elif self.x < -100 and self.y > 100:
                print("ArUco 在 x 負方向且 y 正方向")
                self.vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,self.vehicle.target_system,
                        self.vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111111000) ,1,-1,0,0,0,0,0,0,0,0,0))
                time.sleep(3)

            elif self.x < -100 and self.y < -100:
                print("ArUco 在 x 負方向且 y 負方向")
                self.vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,self.vehicle.target_system,
                        self.vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111111000) ,1,1,0,0,0,0,0,0,0,0,0))
                time.sleep(3)
            
            elif self.x > 100 and self.y < -100:
                print("ArUco 在 x 正方向且 y 負方向")
                self.vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,self.vehicle.target_system,
                        self.vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111111000) ,-1,1,0,0,0,0,0,0,0,0,0))
                time.sleep(3)
            
            elif self.x >100:
                print("ArUco 在 x 正方向且y 方向在標準內")
                self.vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,self.vehicle.target_system,
                        self.vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111111000) ,-1,0,0,0,0,0,0,0,0,0,0))
                time.sleep(3)
            
            elif self.x < -100:
                print("ArUco 在 x 負方向且y 方向在標準內")
                self.vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,self.vehicle.target_system,
                        self.vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111111000) ,1,0,0,0,0,0,0,0,0,0,0))
                time.sleep(3)
            
            elif self.y > 100:
                print("ArUco 在 y 正方向且x 方向在標準內")
                self.vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,self.vehicle.target_system,
                        self.vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111111000) ,0,-1,0,0,0,0,0,0,0,0,0))
                time.sleep(3)
            
            elif self.y < -100:
                print("ArUco 在 y 負方向且x 方向在標準內")
                self.vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,self.vehicle.target_system,
                        self.vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111111000) ,0,1,0,0,0,0,0,0,0,0,0))
                time.sleep(3)
            
            else:
                print("drone 的位置在 ArUco 16 標準內")
                print("持續下降尋找 ArUco 0")
                self.vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,self.vehicle.target_system,
                    self.vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111111000) ,0,0,1,0,0,0,0,0,0,0,0))
                time.sleep(3)
                
        # 確認 ArUco 標記的類型
        elif self.id == self.id_to_find_0:  
            print("Find aruco marker 0")

            # 確認 ArUco 距離飛機距離
            if self.x > 30 and self.y > 30:
                print("ArUco 在 x 正方向且 y 正方向")
                self.vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,self.vehicle.target_system,
                        self.vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111111000) ,-0.3,-0.3,0,0,0,0,0,0,0,0,0))
                time.sleep(3)
                    
            elif self.x < -30 and self.y > 30:
                print("ArUco 在 x 負方向且 y 正方向")
                self.vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,self.vehicle.target_system,
                        self.vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111111000) ,0.3,-0.3,0,0,0,0,0,0,0,0,0))
                time.sleep(3)

            elif self.x < -30 and self.y < -30:
                print("ArUco 在 x 負方向且 y 負方向")
                self.vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,self.vehicle.target_system,
                        self.vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111111000) ,0.3,0.3,0,0,0,0,0,0,0,0,0))
                time.sleep(3)
                    
            elif self.x > 30 and self.y < -30:
                print("ArUco 在 x 正方向且 y 負方向")
                self.vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,self.vehicle.target_system,
                        self.vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111111000) ,-0.3,0.3,0,0,0,0,0,0,0,0,0))
                time.sleep(3)
                    
            elif self.x >30:
                print("ArUco 在 x 正方向且y 方向在標準內")
                self.vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,self.vehicle.target_system,
                        self.vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111111000) ,-0.3,0,0,0,0,0,0,0,0,0,0))
                time.sleep(3)
                    
            elif self.x < -30:
                print("ArUco 在 x 負方向且y 方向在標準內")
                self.vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,self.vehicle.target_system,
                        self.vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111111000) ,0.3,0,0,0,0,0,0,0,0,0,0))
                time.sleep(3)
                    
            elif self.y > 30:
                print("ArUco 在 y 正方向且x 方向在標準內")
                self.vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,self.vehicle.target_system,
                            self.vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111111000) ,0,-0.3,0,0,0,0,0,0,0,0,0))
                time.sleep(3)
                    
            elif self.y < -30:
                print("ArUco 在 y 負方向且x 方向在標準內")
                self.vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,self.vehicle.target_system,
                        self.vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111111000) ,0,0.3,0,0,0,0,0,0,0,0,0))
                time.sleep(3)

            else:
                print("ArUco 位置在標準內")
                print("持續下降到地面")
                self.vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,self.vehicle.target_system,
                        self.vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111111000) ,0,0,0.5,0,0,0,0,0,0,0,0))
                time.sleep(3)

    # 定義找不到 ArUco 的函數
    def not_found_aruco(self):

        # 如果高度小於 1.5m
        if self.altitude <= 1.5:
            print("無人機高度低於", self.altitude, "m,找不到 ArUco")
            print("切換成 LAND 模式")
            self.mode_change_to_land()
            time.sleep(10)
            exit()           
        else:
            print("找不到 ArUco")
    
    # 定義降落模式函數
    def mode_change_to_land(self):

        # 定義降落模式函數
        FLIGHT_MODE = 'LAND'

        # 取得支援的飛航模式
        flight_modes = self.vehicle.mode_mapping()
    
        # 檢查是否支援所需的飛行模式
        if FLIGHT_MODE not in flight_modes.keys():

            print(FLIGHT_MODE, 'is not supported')

            # 中斷
            exit()
        
        # 建立更改模式訊息
        set_mode_message = dialect.MAVLink_command_long_message(
            target_system=self.vehicle.target_system,
            target_component=self.vehicle.target_component,
            command=dialect.MAV_CMD_DO_SET_MODE,
            confirmation=0,
            param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            param2=flight_modes[FLIGHT_MODE],
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        # 捕捉心跳訊息
        message = self.vehicle.recv_match(type=dialect.MAVLink_heartbeat_message.msgname, blocking=True)

        # 將此訊息轉換為字典
        message = message.to_dict()

        # 取得模式ID
        mode_id = message['custom_mode']

        # 取得模式名稱
        flight_mode_names = list(flight_modes.keys())
        flight_mode_ids = list(flight_modes.values())
        flight_mode_index = flight_mode_ids.index(mode_id)
        land_flight_mode_name = flight_mode_names[flight_mode_index ]

        # 改變飛航模式
        self.vehicle.mav.send(set_mode_message)
        while True:

            # 捕獲 COMMAND_ACK 訊息
            message = self.vehicle.recv_match(type=dialect.MAVLink_command_ack_message.msgname, blocking=True)

            # 將此訊息轉換為字典
            message = message.to_dict()

            # 檢查 COMMAND_ACK 是否適用於 DO_SET_MODE
            if message['command'] == dialect.MAV_CMD_DO_SET_MODE:

                # 檢查命令是否被接受
                if message['result'] == dialect.MAV_RESULT_ACCEPTED:

                    print("Changeing mode to", FLIGHT_MODE, "accepted from the vehicle")

                else:

                    print("Changeing mode to", FLIGHT_MODE, "failed")

                break
        
    # 定義導引模式函數
    def mode_change_to_guided(self):

        # 定義導引模式函數
        FLIGHT_MODE = 'GUIDED'

        # 取得支援的飛航模式
        flight_modes = self.vehicle.mode_mapping()
    
        # 檢查是否支援所需的飛行模式
        if FLIGHT_MODE not in flight_modes.keys():

            print(FLIGHT_MODE, 'is not supported')

            # 中斷
            exit()
        
        # 建立更改模式訊息
        set_mode_message = dialect.MAVLink_command_long_message(
            target_system=self.vehicle.target_system,
            target_component=self.vehicle.target_component,
            command=dialect.MAV_CMD_DO_SET_MODE,
            confirmation=0,
            param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            param2=flight_modes[FLIGHT_MODE],
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        # 捕捉心跳訊息
        message = self.vehicle.recv_match(type=dialect.MAVLink_heartbeat_message.msgname, blocking=True)

        # 將此訊息轉換為字典
        message = message.to_dict()

        # 取得模式ID
        mode_id = message['custom_mode']

        # 取得模式名稱
        flight_mode_names = list(flight_modes.keys())
        flight_mode_ids = list(flight_modes.values())
        flight_mode_index = flight_mode_ids.index(mode_id)
        flight_mode_name = flight_mode_names[flight_mode_index ]

        # 改變飛航模式
        self.vehicle.mav.send(set_mode_message)

        while True:

            # 捕獲 COMMAND_ACK 訊息
            message = self.vehicle.recv_match(type=dialect.MAVLink_command_ack_message.msgname, blocking=True)

            # 將此訊息轉換為字典
            message = message.to_dict()

            # 檢查 COMMAND_ACK 是否適用於 DO_SET_MODE
            if message['command'] == dialect.MAV_CMD_DO_SET_MODE:

                # 檢查命令是否被接受
                if message['result'] == dialect.MAV_RESULT_ACCEPTED:

                    print("Changeing mode to", FLIGHT_MODE, "accepted from the vehicle")

                else:

                    print("Changeing mode to", FLIGHT_MODE, "failed")

                break

        # 捕捉心跳訊息
        message = self.vehicle.recv_match(type=dialect.MAVLink_heartbeat_message.msgname, blocking=True)

        # 將此訊息轉換為字典
        message = message.to_dict()

        # 取得模式ID
        mode_id = message['custom_mode']

        # 取得模式名稱
        flight_mode_names = list(flight_modes.keys())
        flight_mode_ids = list(flight_modes.values())
        flight_mode_index = flight_mode_ids.index(mode_id)
        self.flight_mode_name = flight_mode_names[flight_mode_index ]

        # 列印心跳訊息
        print("Now mode is:", self.flight_mode_name)

        # 改變飛航模式
        self.vehicle.mav.send(set_mode_message)

    # 取得航點資訊
    def get_waypoint_info(self):

        # 發送請求命令，請求 MISSION_CURRENT 訊息
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,          
            self.vehicle.target_component,       
            utility.mavlink.MAV_CMD_REQUEST_MESSAGE,  
            0,                              
            utility.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT,  # param1: 訊息ID
            0, 0, 0, 0, 0, 0                # params 2-7: 保留參數
        )

        # 等待並接收 MISSION_CURRENT 訊息
        while True:
            time.sleep(3)
            message = self.vehicle.recv_match(type='MISSION_CURRENT', blocking=True)
            if message:
                if self.flight_mode_name == "AUTO":
                    wp_point = {message.seq}

                    # 將集合轉換成整數
                    self.wp_point = list(wp_point)[0]
                    print(f"當前任務編號: {self.wp_point}")
                    print(f"當前任務總數: {self.wp_point_count}")
                    break

                else:
                    wp_point = {message.seq}

                    # 將集合轉換成整數
                    self.wp_point = list(wp_point)[0]
                    break
    
    # 取得航點總數
    def get_waypoint_count(self):

        # 建立任務請求清單訊息
        message = dialect.MAVLink_mission_request_list_message(target_system=self.vehicle.target_system,
                                                            target_component=self.vehicle.target_component,
                                                            mission_type=dialect.MAV_MISSION_TYPE_MISSION)

        # 向無人機發送訊息
        self.vehicle.mav.send(message)

        # 等待任務計數訊息
        message = self.vehicle.recv_match(type=dialect.MAVLink_mission_count_message.msgname,
                                    blocking = True)

        # 將此訊息轉換為字典
        message = message.to_dict()

        # 取得任務數量
        count = message["count"]
        print("Total mission item count:", count)

        # 建立任務物品列表
        mission_item_list = []

        # 獲得任務清單
        for i in range(count):
            message = dialect.MAVLink_mission_request_int_message(target_system=self.vehicle.target_system,
                                                                target_component=self.vehicle.target_component,
                                                                seq=i,
                                                                mission_type=dialect.MAV_MISSION_TYPE_MISSION)
            
            self.vehicle.mav.send(message)

            # 等待任務計數訊息
            message = self.vehicle.recv_match(type=dialect.MAVLink_mission_item_int_message.msgname,
                                        blocking = True) 
            
            # 將此訊息轉換為字典
            message = message.to_dict()

            # 將任務項目加入清單中
            mission_item_list.append(message)

        for mission_item in mission_item_list:
            print("Seq", mission_item['seq'])
            self.wp_point_count = mission_item['seq'] 

    # 清除原有任務
    def clear_mission(self):

        # 發送清除所有任務的命令
        self.vehicle.mav.mission_clear_all_send(
            self.vehicle.target_system,
            self.vehicle.target_component
        )
        print("All missions cleared")
    
    # 設定 WP_YAW_BEHAVIOR 參數
    def set_wp_yaw_behavior(self,value):

        # WP_YAW_BEHAVIOR 設定參數命令
        self.vehicle.mav.param_set_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            b'WP_YAW_BEHAVIOR',
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_INT8
        )
    
    # 設定偏航角
    def set_yaw(self):

        direction = 0

        """
        param1:目標角度(0為北)
        param2:角速度（度/秒）
        param3:方向（-1:CCW,0:默認,1:CW)
        param4:目標角度類型(0:絕對,1:相對)

        """
        
        if self.aruco_yaw <= 180:
            direction = -1

        else:
            direction = 1
            self.aruco_yaw = 360 - self.aruco_yaw

        # create yaw command(
        message = dialect.MAVLink_command_long_message(target_system=self.vehicle.target_system,
                                                    target_component=self.vehicle.target_component,
                                                    command=dialect.MAV_CMD_CONDITION_YAW,
                                                    confirmation=0,
                                                    param1=self.aruco_yaw,
                                                    param2=0,
                                                    param3=direction,
                                                    param4=1,
                                                    param5=0,
                                                    param6=0,
                                                    param7=0)

        # send yaw command to the vehicle
        self.vehicle.mav.send(message)

        # 等待轉向
        time.sleep(5)
        

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()