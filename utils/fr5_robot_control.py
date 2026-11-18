from fairino import Robot
import time

class RobotControl:
    def __init__(self, robot_ip, name='Fairino'):
        if name == 'Fairino':
            self.robot = Robot.RPC(robot_ip)
            self.robot.SetSpeed(20)
        else:
            raise ValueError("Unsupported robot name. Currently only 'Fairino' is supported.")
        
    def reset(self, joint_pos):
        """Move the robot to its reset position."""
        self.robot.MoveJ(
            joint_pos=joint_pos,
            tool=0,
            user=0,
            vel=100.0,
            acc=100.0,
            ovl=100.0,
            exaxis_pos=[0.0, 0.0, 0.0, 0.0],
            blendT=-1.0,
            offset_flag=0,
            offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )

    def moveL(self, desc_pos):
        """Move the robot linearly to the specified position."""
        self.robot.MoveL(
            desc_pos=desc_pos,
            tool=0,
            user=0,
            vel=100.0,
            acc=100.0,
            ovl=100.0,
            blendR=-1.0,
            blendMode=0,
            exaxis_pos=[0.0, 0.0, 0.0, 0.0],
            search=0,
            offset_flag=0,
            offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            velAccParamMode=0
        )

    def getTCPPose(self):
        """Get the current TCP pose of the robot."""
        return self.robot.GetActualTCPPose(0)
    
    def setDO(self, id, state):
        return self.robot.SetDO(id, state, 0, 0)
    
    def stop(self):
        """Stop the robot."""
        self.robot.CloseRPC()