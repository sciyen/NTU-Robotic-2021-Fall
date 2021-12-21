import numpy as np
import cv2 as cv
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

        self.mbTc = self.img_loader.mbTc

    def __calc_principal_angle(self, cnt):
        m = cv.moments(cnt)
        centroid = np.array([m.m10/m.m00, m.m01/m.m00])
        pa = np.atan2(2*m.mu11, m.mu20-m.mu02) / 2 * 180 / np.pi
        return centroid, pa

    def __find_largest_object(self, img, mask, show, line_length=100, min_criteria=100):
        ret, thresh = cv.threshold(img, 180, 255, cv.THRESH_BINARY)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel, iterations=1)

        # retval, labels, stats, centroids = cv.connectedComponentsWithStats(
        #    mask, connectivity=4)
        # largest_idx = np.argmax(stats[4])  # index 4 for area
        #obj = (labels == largest_idx).astype(np.uint8)

        cnts, _ = cv.findContours(
            mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        max_cnt = max(cnts, key=cv.contourArea)

        # Check if it is a valid object
        if cv.contourArea(max_cnt) < min_criteria:
            return None, None, None

        centroid, pa = self.__calc_principal_angle(cnt=max_cnt)

        if show:
            cv.drawContours(img, [max_cnt], 0, (255, 0, 255), 5)
            pt2 = centroid + \
                np.array([np.cos(pa), np.sin(pa)], dtype=int) * line_length
            cv.arrowedLine(img, centroid, pt2, (0, 255, 255), 2)
        return max_cnt, centroid, pa

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

    def __transform_img_frame_to_arm_frame(self, im_pt):
        depth = None
        c_pt = self.img_loader.get_c_pt_im(im_pt, depth)
        b_pt = self.mbTc @ c_pt
        return b_pt

    def run(self):
        cap = cv.VideoCapture(0)
        if not cap.isOpened():
            print("Cannot open camera")
            exit()
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
        frame = self.img_loader.undistort_image(frame)

        # Find the largest object and set it as base
        cnt, im_pt, target_orien = self.__find_largest_object(
            img=frame, mask=None)
        b_target = self.__transform_img_frame_to_arm_frame(im_pt)
        mask = cnt.copy()

        # Iteration until no object can be found
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
            frame = self.img_loader.undistort_image(frame)

            cnt, im_pt, obj_orien = self.__find_largest_object(
                img=frame, mask=mask)
            if cnt is None:
                break
            b_obj = self.__transform_img_frame_to_arm_frame(im_pt)
            self.__grab_and_release(b_obj, obj_orien, b_target, target_orien, )

        cap.release()
        cv.destroyAllWindows()


def main():
    gripper = Gripper()
    gripper.run()


if __name__ == "__main__":
    main()
