import threading
import time
import cv2
import numpy as np
from utils import yolo_model

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


reset_pos = [90.000, -140.000, 140.000, -180.000, -90.000, -90.000]


class CutWatermelonTask:
    def __init__(self, robot, camera, detector):
        self.robot = robot
        self.camera = camera
        self.detector = detector
        self.stop_threads = False

        self.current_target = None
        self.current_target_dist = None
        self.target_lock = threading.Lock()

        # ---- 两态状态机 ----
        self.task_state = "PATROLLING"   # "PATROLLING" | "STOP"
        self.task_state_lock = threading.Lock()

        # ROS2
        rclpy.init(args=None)
        self.ros_node = Node("watermelon_detector_node")

        self.publisher = self.ros_node.create_publisher(String, "/chatter", 10)

        self.patrol_state_sub = self.ros_node.create_subscription(
            String,
            "/patrol_state",
            self._patrol_state_callback,
            10
        )

        # ROS spin 独立线程
        threading.Thread(target=lambda: rclpy.spin(self.ros_node), daemon=True).start()

        opt = yolo_model.parse_args()
        device, use_half = yolo_model.setup_device(opt)
        self.opt = opt
        self.device = device
        self.use_half = use_half

    # ------------------------------------------------------------------
    # ROS2 回调
    # ------------------------------------------------------------------
    def _patrol_state_callback(self, msg: String):
        """
        /patrol_state == "STOP"   → 切换到 STOP
        /patrol_state == "PATROL" → 切换到 PATROLLING
        """
        state = msg.data
        print(f"[PatrolState] {state}")

        with self.task_state_lock:
            if state == "STOP" and self.task_state == "PATROLLING":
                self.task_state = "STOP"
                print("[TaskState] → STOP")
            elif state == "PATROL" and self.task_state == "STOP":
                self.task_state = "PATROLLING"
                print("[TaskState] → PATROLLING")

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.publisher.publish(msg)
        print(f"[ROS Publish] {status}")

    # ------------------------------------------------------------------
    # 相机线程：只负责检测 + 发布指令 + 更新坐标
    # ------------------------------------------------------------------
    def camera_thread(self):
        prev_t = time.time()

        while not self.stop_threads:
            img_color, img_depth, aligned_depth_frame, aligned_color_frame, matrix = \
                self.camera.get_frames()
            if not aligned_color_frame or not aligned_depth_frame:
                continue

            pred = self.detector.predict(
                source=img_color,
                verbose=False,
                device=self.device,
                conf=self.opt.conf,
                iou=self.opt.iou,
                imgsz=256,
            )

            res = pred[0]
            boxes = res.boxes
            watermelon_target = []

            for b in boxes:
                cls_id = int(b.cls[0].item())
                name = res.names.get(cls_id, str(cls_id))
                if name != 'watermelon':
                    continue

                x1, y1, x2, y2 = b.xyxy[0].tolist()
                conf = float(b.conf[0].item())
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                depth = self.camera.median_depth(aligned_depth_frame, cx, cy, window_size=5)
                depth_top = self.camera.median_depth(aligned_depth_frame, cx, int(y1), window_size=5)

                cv2.rectangle(img_color, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(img_color, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(img_color, f'{name} {conf:.2f}',
                            (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if depth > 0:
                    X, Y, Z = self.camera.deproject_pixel_to_point(matrix, cx, cy, depth)
                    cv2.putText(img_color, f'X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f}',
                                (int(x1), int(y1) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                    if -0.05 <= X <= 0.1 and Y != 0 and Z < 0.6:
                        Xt, Yt, Zt = self.camera.deproject_pixel_to_point(matrix, cx, int(y1), depth_top)
                        dist = abs(Y - Yt) * 1000
                        watermelon_target.append(((-X * 1000, -Z * 1000, -Y * 1000), dist))

            watermelon_target.sort(key=lambda item: item[0][0], reverse=True)

            with self.task_state_lock:
                current_state = self.task_state

            if watermelon_target:
                best_target, best_dist = watermelon_target[-1]

                # 不管什么状态，持续更新坐标供机械臂使用
                with self.target_lock:
                    self.current_target = best_target
                    self.current_target_dist = best_dist

                if current_state == "PATROLLING":
                    # 巡逻中检测到西瓜 → 持续发触发词，等 /patrol_state 回调切换状态
                    self.publish_status('watermelon')
                # STOP 状态下相机只更新坐标，不重复发布

            else:
                # 未检测到西瓜
                with self.target_lock:
                    self.current_target = None
                    self.current_target_dist = None

                if current_state == "PATROLLING":
                    self.publish_status('notwatermelon')
                # STOP 状态下即使西瓜消失也不发 notwatermelon，防止小车重新启动

            # FPS + 状态显示
            now = time.time()
            fps = 1.0 / (now - prev_t) if now > prev_t else 0.0
            prev_t = now
            cv2.putText(img_color, f'FPS:{fps:.1f}  State:{current_state}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow('Cut Watermelon Task', img_color)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop_threads = True
                break

        self.camera.stop()
        cv2.destroyAllWindows()
        self.ros_node.destroy_node()
        rclpy.shutdown()
        print("Camera thread stopped.")

    # ------------------------------------------------------------------
    # 机械臂线程：等 STOP 状态 + 坐标稳定后执行，完成后恢复 PATROLLING
    # ------------------------------------------------------------------
    def robot_thread(self):
        self.robot.reset(reset_pos)
        self.robot.setDO(8, 0)

        STABLE_FRAMES  = 10    # 等待稳定帧数，可按实际帧率调整
        STABLE_TIMEOUT = 5.0   # 超时保底（秒）

        while not self.stop_threads:
            # 等待 STOP 状态
            with self.task_state_lock:
                current_state = self.task_state
            if current_state != "STOP":
                time.sleep(0.1)
                continue

            # ---- 等待坐标稳定（跳过刚停车的前 N 帧）----
            print(f"[Robot] 小车已停，等待 {STABLE_FRAMES} 帧坐标稳定...")
            stable_count  = 0
            stable_target = None
            stable_dist   = None
            wait_start    = time.time()

            while stable_count < STABLE_FRAMES:
                if time.time() - wait_start > STABLE_TIMEOUT:
                    print("[Robot] 等待超时，使用当前坐标")
                    break

                with self.target_lock:
                    t = self.current_target
                    d = self.current_target_dist

                if t is not None:
                    stable_target = t
                    stable_dist   = d
                    stable_count += 1
                    print(f"[Robot] 稳定帧 {stable_count}/{STABLE_FRAMES}: {t}")
                else:
                    # 西瓜短暂消失则重置，确保是连续稳定的帧
                    stable_count = 0
                    print("[Robot] 西瓜消失，重置稳定帧计数")

                time.sleep(0.1)

            # 超时或西瓜始终未出现
            if stable_target is None:
                print("[Robot] 未能获取有效坐标，放弃本次抓取，恢复巡逻")
                with self.target_lock:
                    self.current_target      = None
                    self.current_target_dist = None
                with self.task_state_lock:
                    self.task_state = "PATROLLING"
                self.publish_status('notwatermelon')
                continue

            target      = stable_target
            target_dist = stable_dist
            print(f"[Robot] 坐标稳定，最终使用: {target}, dist={target_dist:.1f}")

            # ---- 执行抓取 ----
            try:
                x_incr, y_incr, z_incr = target
                x_c, y_c, z_c = 10, -20, 80
                x_t, y_t, z_t = 0, -105, 200

                errcode, tcp_pose = self.robot.getTCPPose()
                if errcode != 0:
                    print(f"[Robot] getTCPPose 失败 errcode={errcode}，重试...")
                    time.sleep(0.1)
                    continue

                new_tcp_pose_0 = [tcp_pose[0], tcp_pose[1] - 50, tcp_pose[2],
                                  -90.0, -90.0, 180.0]
                new_tcp_pose_1 = [tcp_pose[0] + x_incr + x_c + x_t,
                                  tcp_pose[1] - 50,
                                  tcp_pose[2] + z_incr + z_c + target_dist - z_t,
                                  -90.0, -90.0, 180.0]
                new_tcp_pose_2 = [tcp_pose[0] + x_incr + x_c,
                                  tcp_pose[1] + y_incr + y_c + 25,
                                  tcp_pose[2] + z_incr + z_c + target_dist - z_t + 20,
                                  -90.0, -90.0, 180.0]

                print('[Robot] 开始抓取...')
                self.robot.moveL(new_tcp_pose_0)
                self.robot.moveL(new_tcp_pose_1)
                self.robot.moveL(new_tcp_pose_2)
                time.sleep(1)
                self.robot.setDO(8, 1)
                time.sleep(2.5)
                self.robot.setDO(8, 0)
                time.sleep(2.5)
                self.robot.moveL(new_tcp_pose_1)

            except Exception as e:
                print(f"[Robot] 抓取异常: {e}")

            finally:
                self.robot.reset(reset_pos)
                time.sleep(3)
                with self.target_lock:
                    self.current_target      = None
                    self.current_target_dist = None
                with self.task_state_lock:
                    self.task_state = "PATROLLING"
                self.publish_status('notwatermelon')
                print("[TaskState] → PATROLLING，等待小车重新启动")

        self.robot.setDO(8, 0)
        self.robot.stop()
        print("Robot thread stopped.")

    # ------------------------------------------------------------------
    def start(self):
        t1 = threading.Thread(target=self.camera_thread)
        t2 = threading.Thread(target=self.robot_thread)
        t1.start()
        t2.start()
        t1.join()
        t2.join()
        print('All threads have been stopped. Task completed.')
