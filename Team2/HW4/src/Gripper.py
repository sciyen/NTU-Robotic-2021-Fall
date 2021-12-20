import numpy as np
from ImageLoader import ImageLoader


class ArmControl:
    def __init__(self) -> None:
        pass

    def release(self):
        pass

    def grab(self):
        pass

    def move_to_pose(self, a_pos, a_orien):
        """Move to a specific pose
        @Params:
            a_pos: 1x4 matrix in homogenous form, describing position
            a_orien: 1x3 matrix, describing Euler angles of the orientation 
                of the end-effector
        """
        pass


class Gripper:
    def __init__(self, calibrate_extrinsic) -> None:
        """Calibration process (or load previous calibrated file), including
        1. intrinsic calibration if not done yet.
        2. extrinsic calibrarion (control by the flag)
        """
        self.img_loader = ImageLoader(
            recalibrate_extrinsic=calibrate_extrinsic)

        self.arm = ArmControl()

    def __calc_principal_angle(self):
        pass

    def __find_largest_object(self):
        pass

    def __grab_and_release(self, a_obj, a_psi_obj, a_target, a_psi_tar, zoffset_after_release):
        a_ready_pos = a_obj.copy()
        a_ready_pos[2, 0] += zoffset_after_release
        # TODO: the other angles need to be determined
        a_grab_orientation = np.array([0, 0, a_psi_obj], ndmin=2).T
        # Move to target
        self.arm.move_to_pose(a_ready_pos, a_grab_orientation)
        # Move downward
        self.arm.move_to_pose(a_obj, a_grab_orientation)
        # Grab
        self.arm.grab()

        # Go to target position
        a_ready_pos = a_target.copy()
        a_ready_pos[2, 0] += zoffset_after_release
        a_release_orientation = np.array([0, 0, a_psi_tar], ndmin=2).T
        self.arm.move_to_pose(a_ready_pos, a_release_orientation)
        # Check the arm has released
        self.arm.release()
        # Move down
        self.arm.move_to_pose(a_target, a_release_orientation)

    def __transform_img_frame_to_arm_frame(self):
        pass

    def run(self):
        # Find the largest object and set it as base
        cnt, im_pt, target_orien = self.__find_largest_object(mask=None)
        a_target = self.__transform_img_frame_to_arm_frame()
        mask = cnt.copy()

        # Iteration until no object can be found
        while True:
            cnt, im_pt, obj_orien = self.__find_largest_object(mask=mask)
            a_obj = self.__transform_img_frame_to_arm_frame()
            self.__grab_and_release(a_obj, obj_orien, a_target, target_orien, )
