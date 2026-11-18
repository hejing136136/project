import threading
import time
import cv2
import numpy as np
from utils import yolo_model

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


reset_pos = [90.000, -120.000, 120.000, -180.000, -90.000, -90.000]
# reset_pos = [90.000, -120.000, 120.000, 0, 90.000, 90.000]
# reset_pos = [90.000, -70.000, 140.000, -250.000, -90.000, -90.000]



class CutWatermelonTask:
    def __init__(self, robot, camera, detector):
        self.robot = robot
        self.camera = camera
        self.detector = detector
        self.stop_threads = False
        self.current_target = None
        self.current_target_dist = None
        self.target_lock = threading.Lock()

        self.grabbing = False
        self.grabbing_lock = threading.Lock()

        #ROS2 Init
        rclpy.init(args=None)
        self.ros_node = Node("watermelon_detector_node")
        self.publisher = self.ros_node.create_publisher(
            String,
            "/chatter",
            10
        )

        self.last_status = None

        opt = yolo_model.parse_args()
        device, use_half = yolo_model.setup_device(opt)
        self.opt = opt
        self.device = device
        self.use_half = use_half



    def publish_status(self, status):
        # if status != self.last_status:
        msg = String()
        msg.data = status
        self.publisher.publish(msg)
        self.last_status = status
        print(f"ROS Published: {status}")

    def camera_thread(self):
        prev_t = time.time()
        while not self.stop_threads:
            img_color, img_depth, aligned_depth_frame, aligned_color_frame, matrix = self.camera.get_frames()
            if not aligned_color_frame or not aligned_depth_frame:
                continue

            pred = self.detector.predict(
                source=img_color,
                verbose=False,
                device=self.device,
                conf=self.opt.conf,
                iou=self.opt.iou,
                imgsz = 256,
            )

            res = pred[0]
            boxes = res.boxes

            detected = False
            watermelon_target = []
            for b in boxes:
                cls_id = int(b.cls[0].item())
                name = res.names.get(cls_id, str(cls_id))
                if name == 'watermelon':
                    x1, y1, x2, y2 = b.xyxy[0].tolist()
                    conf = float(b.conf[0].item())

                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    depth = self.camera.median_depth(aligned_depth_frame, cx, cy, window_size=5)
                    depth_top = self.camera.median_depth(aligned_depth_frame, cx, int(y1), window_size=5)
                    label = f'{name} {conf:.2f}'
                    cv2.rectangle(img_color, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.circle(img_color, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.circle(img_color, (cx, int(y1)), 5, (0, 0, 255), -1)
                    cv2.putText(img_color, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    if depth > 0:
                        X, Y, Z = self.camera.deproject_pixel_to_point(matrix, cx, cy, depth)
                        cv2.putText(img_color, f'X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f}', (int(x1), int(y1) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        if X >= -0.0 and X <= 0.1 and Y != 0 and Z != 0:
                            with self.target_lock:
                                Xt, Yt, Zt = self.camera.deproject_pixel_to_point(matrix, cx, int(y1), depth_top)
                                dist = abs(Y - Yt)*1000
                                watermelon_target.append(((-X*1000, -Z*1000, -Y*1000), dist))
            
            watermelon_target.sort(key=lambda item: item[0][0], reverse=True)
            if watermelon_target:
                detected = True
                self.current_target, self.current_target_dist = watermelon_target[-1]
            else:
                self.current_target = None
                self.current_target_dist = None
            # print(self.current_target, self.current_target_dist)

            now = time.time()
            fps = 1.0 / (now - prev_t) if now > prev_t else 0.0
            prev_t = now
            cv2.putText(img_color, f'FPS: {fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.imshow('Cut Watermelon Task - Camera View', img_color)

            with self.grabbing_lock:
                grabbing_now = self.grabbing
            if detected:
                self.publish_status('watermelon')
            elif grabbing_now:
                self.publish_status('watermelon')
            else:
                self.publish_status('notwatermelon')
                with self.target_lock:
                    self.current_target = None
            # if not detected:
            #     with self.target_lock:
            #         self.current_target = None
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop_threads = True
                break
        
        self.camera.stop()
        cv2.destroyAllWindows()
        self.ros_node.destroy_node()
        rclpy.shutdown()
        print("Camera thread stopped.")

    def robot_thread(self):
        self.robot.reset(reset_pos)
        errcode, tcp_pose = self.robot.getTCPPose()
        if errcode == 0:
            # 中间点0, y方向靠近西瓜
            reset_pose_0 = [
                tcp_pose[0],
                tcp_pose[1] - 10,
                tcp_pose[2] - 50,
                -90.000,
                -90.000,
                180.000
                # tcp_pose[3],
                # tcp_pose[4],
                # tcp_pose[5]
            ]
            self.robot.moveL(reset_pose_0)
        self.robot.setDO(8,0)

        while not self.stop_threads:
            target = None
            with self.target_lock:
                if self.current_target is not None:
                    target = self.current_target
                    target_dist = self.current_target_dist

            if target is not None:
                with self.grabbing_lock:
                    self.grabbing = True
                
                try:
                    x_incr, y_incr, z_incr = target
                    x_c, y_c, z_c = [10, -20, 80]
                    x_t, y_t, z_t = [0, -105, 200]
                    errcode, tcp_pose = self.robot.getTCPPose()
                    if errcode == 0:
                        # 中间点0, y方向靠近西瓜
                        new_tcp_pose_0 = [
                            tcp_pose[0],
                            tcp_pose[1] - 100,
                            tcp_pose[2],
                            -90.000,
                            -90.000,
                            180.000
                            # tcp_pose[3],
                            # tcp_pose[4],
                            # tcp_pose[5]
                        ]

                        # 中间点1, 剪刀对准藤蔓
                        new_tcp_pose_1 = [
                            tcp_pose[0] + x_incr + x_c + x_t,
                            tcp_pose[1] -100,
                            tcp_pose[2] + z_incr + z_c + self.current_target_dist - z_t,
                            -90.000,
                            -90.000,
                            180.000
                            # tcp_pose[3],
                            # tcp_pose[4],
                            # tcp_pose[5]
                        ]

                        # 剪切点
                        new_tcp_pose_2 = [
                            tcp_pose[0] + x_incr + x_c,
                            tcp_pose[1] + y_incr + y_c + 25,#TODO: 加入剪刀后设置距离远离
                            tcp_pose[2] + z_incr + z_c + self.current_target_dist - z_t + 20,
                            -90.000,
                            -90.000,
                            180.000
                            # tcp_pose[3],
                            # tcp_pose[4],
                            # tcp_pose[5]
                        ]
                        print('开始抓取...')
                        self.robot.moveL(new_tcp_pose_0)
                        self.robot.moveL(new_tcp_pose_1)
                        self.robot.moveL(new_tcp_pose_2)
                        time.sleep(1)
                        self.robot.setDO(8,1)
                        time.sleep(2.5)
                        self.robot.setDO(8,0)
                        time.sleep(2.5)
                        # TODO: 添加末端执行

                        self.robot.moveL(new_tcp_pose_1)
                    else:
                        time.sleep(0.1)

                    # self.robot.reset(reset_pos)
                    self.robot.moveL(reset_pose_0)
                    time.sleep(3)
                finally:
                    with self.grabbing_lock:
                        self.grabbing = False
                    with self.target_lock:
                        self.current_target = None
                        self.current_target_dist = None

            else:
                time.sleep(0.1)
        self.robot.setDO(8,0)
        self.robot.stop()
        print("Robot thread stopped.")


    def start(self):
        t1 = threading.Thread(target=self.camera_thread)
        t2 = threading.Thread(target=self.robot_thread)
        t1.start()
        t2.start()
        t1.join()
        t2.join()
        print('All threads have been stopped. Task completed.')
