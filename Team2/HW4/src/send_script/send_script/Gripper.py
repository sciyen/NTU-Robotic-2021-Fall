import numpy as np
import cv2 as cv
import ImageProcessing
from ImageLoader import ImageLoader


class ArmControl:
    def __init__(self, send_script, set_io) -> None:
        self.send_script = send_script
        self.set_io = set_io

    def release(self):
        self.set_io(0.0)

    def grab(self):
        self.set_io(1.0)

    def move_to_pose(self, pose):
        """Move to a specific pose
        @Params:
            a_pos: 1x4 matrix in homogenous form, describing position
            a_orien: 1x3 matrix, describing Euler angles of the orientation 
                of the end-effector
        """
        if pose[2] < 105:
            pose[2] = 105
            print("Height too low: ", pose[2])
        targetP1 = '{0}, {1}, {2}, {3}, {4}, {5}'.format(
            pose[0], pose[1], pose[2], pose[3], pose[4], pose[5])
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        self.send_script(script1)

    def take_photo(self):
        self.send_script("Vision_DoJob(job1)")


class Gripper:
    def __init__(self, arm) -> None:
        """Calibration process (or load previous calibrated file), including
        1. intrinsic calibration if not done yet.
        2. extrinsic calibrarion (control by the flag)
        """

        self.arm = arm
        self.img_loader = ImageLoader()
        self.mbTc = self.img_loader.mbTc

    def __grab_and_release(self, b_obj_state, b_target_state, zoffset_after_release) -> None:
        """Perform grab and release task.

        initial  obj     target
        +--------++------++
                 |^      |^
                 v|      v|
                 grab    release 

        The states are defined as follow:
        [x, y, z, alpha, beta, gamma] in base frame (the base of the arm)

        @Params:
            b_obj_state: state of object to grab
            b_target_state: state of object to release
            zoffset_after_release: additional height before arriving
        """
        # ======= Move to object =======
        b_ready_state = b_obj_state.copy()
        b_ready_state[2] += zoffset_after_release
        self.arm.move_to_pose(b_ready_state)
        self.arm.release()
        # Move down
        self.arm.move_to_pose(b_obj_state)
        self.arm.grab()
        # Move up
        self.arm.move_to_pose(b_ready_state)

        # ======= Move to target =======
        b_ready_state = b_target_state.copy()
        b_ready_state[2] += zoffset_after_release
        self.arm.move_to_pose(b_ready_state)
        # Move down
        self.arm.move_to_pose(b_target_state)
        self.arm.release()
        # Move up
        self.arm.move_to_pose(b_ready_state)

    def __transform_img_frame_to_arm_frame(self, im_pt) -> np.ndarray:
        """Transform from image frame to arm frame

        @Params:
            im_pt: [x, y] in image frame

        @Returns:
            b_pt: [x, y, z] in base frame
        """
        depth = None
        c_pt = self.img_loader.get_c_pt_im(im_pt, depth)
        b_pt = self.mbTc @ c_pt
        return b_pt.T[0, :3]

    def __transform_img_orient_to_arm_orient(self, im_psi):
        orientation = np.array([-180, 0, 135])
        # The z-axis's positive direction of image coordinate and
        # the arm coordinate when alpha=-180, beta=0 and gamma=135
        # are the same, so the offseted angle can be added directly.
        # ---------> im_x   -----------> b_x
        # |                 | gamma=135
        # |                 |
        # |                 |
        # v im_y            v b_y
        orientation[2] += im_psi
        return orientation

    def run(self, img, manual=False):
        # Find the largest object and set it as base
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        im_pt, im_pa, cnt = ImageProcessing.find_largest_object(
            img=img, mask=None)
        mask = cv.drawContours(mask, [cnt], -1, 255)

        # Transform to base frame
        b_target_pos = self.__transform_img_frame_to_arm_frame(im_pt)
        b_target_ori = self.__transform_img_orient_to_arm_orient(im_pa)

        if manual:
            input("Press any key to continue")

        height_offset = np.array([[0, 0, 15]]).T
        # Iteration until no object can be found
        for i in range(2):
            im_pt, im_pa, cnt = ImageProcessing.find_largest_object(
                img=img, mask=mask)
            if cnt is None:
                break

            # Update the mask
            mask = cv.drawContours(mask, [cnt], -1, 255)

            # Transform to base frame
            b_obj_pos = self.__transform_img_frame_to_arm_frame(im_pt)
            b_obj_ori = self.__transform_img_orient_to_arm_orient(im_pa)
            b_obj_state = np.hstack([b_obj_pos, b_obj_ori])

            # The tower grows
            b_target_pos += height_offset * i
            b_target_state = np.hstack([b_target_pos, b_target_ori])

            if manual:
                input("Press any key to continue")

            self.__grab_and_release(
                b_obj_state, b_target_state, zoffset_after_release=50)

        cv.destroyAllWindows()


def main():
    gripper = Gripper()
    gripper.run()


if __name__ == "__main__":
    main()
