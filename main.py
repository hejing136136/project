from utils.fr5_robot_control import RobotControl
from utils.camera_control import CameraControl
#from utils.cut_task import CutWatermelonTask
from utils.cut_task_patrol_sub import CutWatermelonTask
from utils import yolo_model

robot_ip = '192.168.58.2'
reset_pos = [90.000, -120.000, 120.000, -180.000, -90.000, -90.000]
reset_pos = [90.000, -120.000, 120.000, 0, 90.000, 90.000]
desc_pos1 = [160.000, -290.000, 600.000, -90.000, -90.000, 180.000]
desc_pos2 = [160.000, -400.000, 600.000, -90.000, -90.000, 180.000]


if __name__ == "__main__":
    robot = RobotControl(robot_ip)
    camera = CameraControl()
    opt = yolo_model.parse_args()
    device, use_half = yolo_model.setup_device(opt)
    fa_on = yolo_model.check_flash_attention()
    model = yolo_model.load_model(opt.model, device, use_half, opt.conf, opt.iou)
    cuttask = CutWatermelonTask(robot, camera, model)

    cuttask.start()
