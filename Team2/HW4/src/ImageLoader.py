import numpy as np
import cv2 as cv
import os

from numpy.core.fromnumeric import ndim


class Calibration:
    def __init__(self) -> None:
        pass

    def load_intrinsic_from_file(self, path):
        s = cv.FileStorage(path, cv.FileStorage_READ)
        self.K = s.getNode('mtx').mat()
        self.dist = s.getNode('dist').mat()
        s.release()

    def write_intrinsic_to_file(self, path):
        s = cv.FileStorage(path, cv.FileStorage_WRITE)
        s.write('mtx', self.K)
        s.write('dist', self.dist)
        s.release()

    def load_extrinsic_from_file(self, path):
        s = cv.FileStorage(path, cv.FileStorage_READ)
        self.rvec = s.getNode('rvec').mat()
        self.tvec = s.getNode('tvec').mat()
        s.release()

    def write_extrinsic_to_file(self, path):
        s = cv.FileStorage(path, cv.FileStorage_WRITE)
        s.write('rvec', self.rvec)
        s.write('tvec', self.tvec)
        s.release()

    def calibrate_intrinsic(self, size_of_tile=0.023):
        """Record a video, extract the chessboard corners, and 
        calibrate it with pairs of points.

        @Params:
            size_of_tile: size of the tile in the chessboard

        @Returns:
            K: 3x3 matrix
            dist: distortion parameters 
        """
        # Defining the dimensions of checkerboard
        CHECKERBOARD = (6, 9)

        # termination criteria
        criteria = (cv.TERM_CRITERIA_EPS +
                    cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Defining the 3D world coordinates of the chessboard
        objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0],
                                  0:CHECKERBOARD[1]].T.reshape(-1, 2) * size_of_tile

        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.

        # Load from camera
        cap = cv.VideoCapture(0)
        if not cap.isOpened():
            print("Cannot open camera")
            exit()
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            """
            Calibrarion
            """
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv.findChessboardCorners(
                gray, CHECKERBOARD, cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FAST_CHECK + cv.CALIB_CB_NORMALIZE_IMAGE)

            """
            If desired number of corner are detected, we refine the pixel coordinates 
            and display them on the image.
            """
            if ret == True:
                objpoints.append(objp)
                # refining pixel coordinates for given 2d points.
                corners2 = cv.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), criteria)

                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv.drawChessboardCorners(
                    frame, CHECKERBOARD, corners2, ret)

                cv.imshow('Intrinsic calibration', img)
            if cv.waitKey(500) == ord('q'):
                cv.destroyAllWindows()
                break
        cap.release()

        """
        Performing camera calibration by passing the value of known 3D points (objpoints)
        and corresponding pixel coordinates of the detected corners (imgpoints)
        """
        print('{} images were found'.format(len(imgpoints)))
        print("Calibrating...")
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None)
        print("intrinsic params: ", mtx)
        print("distortion params: ", dist)

        self.K = mtx
        self.dist = dist

    def __move_arm(self, a_pt):
        pass

    def __get_marker_pos(self, img):
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        # ret, thresh = cv.threshold(
        #    hsv[:, :, 0], 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)
        ret, thresh = cv.threshold(
            hsv[:, :, 0], 35, 255, cv.THRESH_BINARY_INV)
        ret, thresh = cv.threshold(
            thresh, 25, 255, cv.THRESH_BINARY)

        # Noise removal
        kernel = np.ones((3, 3), np.uint8)
        opening = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel, iterations=2)

        cnts = cv.findContours(thresh, cv.RETR_EXTERNAL,
                               cv.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        max_cnt = max(cnts, key=cv.contourArea)
        fitted_circle = cv.fitEllipse(max_cnt)
        cv.drawContours(img, [max_cnt], -1, (255, 0, 0), -1)
        cv.ellipse(img, fitted_circle, (0, 255, 255), 2)
        print('fitted center of the ball: ', fitted_circle[0])

        cv.imshow('img', img)
        cv.imshow('thresh', opening)
        cv.waitKey(100)

    def calibrate_extrinsic(self, scanning_size=(100, 100, 0), stride=(10, 10, 10)):
        """Procedures:
        1. Move robot to several positions, 
        2. Capture the image,
        3. Extract feature (convert into HSV and find the colored ball)
        4. Calculate the centroid of the ball
        5. Repeat from 1~4 and collect the 3D-2D points pairs, finally,
            solve the extrinsic parameters with solvePnP()

        @Returns:
            ext_params: 2x3 matrix
        """
        cap = cv.VideoCapture(0)
        if not cap.isOpened():
            print("Cannot open camera")
            exit()

        for z in range(0, scanning_size[2], stride[2]):
            # Generating scanning queue
            x = np.arange(-scanning_size[0]//2, scanning_size[0]//2, stride[0])
            y = np.arange(0, scanning_size[1], stride[1])
            xx, yy = np.meshgrid(x, y)
            mesh = np.array([xx, yy]).transpose([1, 2, 0])
            mesh[1::2] = np.flip(mesh[1::2], axis=1)
            len_sample = mesh.shape[0] * mesh.shape[1]
            mesh = np.reshape(mesh, [len_sample, 2])
            mesh = np.hstack([mesh, z * np.ones((1, len_sample)).T])

            img_points = []
            for pt in mesh:
                # Move the arm to the specific position
                self.__move_arm(pt)

                # TODO: wait for arm to arrive
                # Capture frame-by-frame
                ret, frame = cap.read()
                if not ret:
                    print("Can't receive frame (stream end?). Exiting ...")
                    break
                im_pt = self.__get_marker_pos(frame)
                img_points.append(im_pt)

            # SolvePnP
            # https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d
            retval, rvec, tvec = cv.solvePnP(mesh, img_points, self.K, self.dist,
                                             flags=cv.SOLVEPNP_ITERATIVE)
        cap.release()
        cv.destroyAllWindows()
        self.rvec = rvec
        self.tvec = tvec
        return rvec, tvec

    def get_mbTc(self):
        mcTb = np.eye(4)

        # The rvec and tvec transform the object frame to camera frame
        mcTb[:3, :3], jacobian = cv.Rodrigues(self.rvec)
        mcTb[:3, 3] = self.tvec

        mbTc = np.linalg.inv(mcTb)
        return mbTc


class ImageLoader:
    """This class is responsible for providing calibrated images
    """

    def __init__(self, recalibrate_extrinsic=True, path_to_intrinsic_param='intrinsic_params.yaml', path_to_extrinsic_param='extrinsic_params.yaml') -> None:
        calibration = Calibration()
        if os.path.isfile(path_to_intrinsic_param):
            print("Load intrinsic parameters.")
            calibration.load_intrinsic_from_file(path=path_to_intrinsic_param)
        else:
            print(
                "No previous calibration file were found, start to calibrate intrinsic parameters.")
            calibration.calibrate_intrinsic()
            calibration.write_intrinsic_to_file(path=path_to_intrinsic_param)

        if recalibrate_extrinsic:
            input("Press any key to start extrinsic parameters calibration")
            calibration.calibrate_extrinsic()
            calibration.write_extrinsic_to_file(path=path_to_extrinsic_param)
        else:
            print("Load extrinsic from file")
            calibration.load_extrinsic_to_file(path=path_to_extrinsic_param)

        self.K = calibration.K
        self.dist = calibration.dist
        self.mbTc = calibration.get_mbTc()

    def undistort_image(self, img):
        dst = cv.undistort(img, self.K, self.dist)
        return dst

    def get_c_pt_im(self, im_pt, depth):
        c_homo_pt = np.linalg.inv(
            self.K) @ np.array([im_pt[0], im_pt[1], 1], ndmin=2).T
        c_pt = depth * c_homo_pt
        return np.vstack([c_pt, 1])


def main():
    img_loader = ImageLoader()


if __name__ == "__main__":
    main()
