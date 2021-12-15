import os
import sys
import math
import numpy as np
import cv2 as cv

class ImageProcessing():
    
    def __init__(self,filename):
        # filename = os.path.abspath(sys.argv[1])
        self.filename = filename
        self.srcIMG = cv.imread(filename,cv.IMREAD_GRAYSCALE)
        self.workIMG = cv.imread(filename,cv.IMREAD_GRAYSCALE)
        self.resultIMG = cv.imread(filename,cv.IMREAD_COLOR)
        self.kernel = np.ones((3,3), np.uint8)
    
    def __maxmin__(self,mat):

        max = 0
        min = 255

        for row in range(mat.shape[0]):
            for col in range(mat.shape[1]):
                if mat[row][col] > max:
                    max = mat[row][col]
                if mat[row][col] < min:
                    min = mat[row][col]
        return max,min
   
    def pre_Processing(self):
        ret,self.workIMG  = cv.threshold(self.workIMG,180,255,cv.THRESH_BINARY)
        cv.erode(self.workIMG,self.kernel,self.workIMG)
        cv.dilate(self.workIMG,self.kernel,self.workIMG)

    def Processing(self):
            self.pre_Processing()

            # self.visulaize(self.srcIMG,self.workIMG)
            # return
            # print(self.workIMG.shape)

            pixel = 0
            for row in range(self.workIMG.shape[0]):
                for col in range(self.workIMG.shape[1]):
                    if(self.workIMG[row,col] == 255):
                        # print(row,col)
                        pixel = pixel + 1
                        self.regionGrowing(row,col,pixel)

            # self.visulaize(self.workIMG)
            region_num, _ = self.__maxmin__(self.workIMG)

            self.contour,_ = cv.findContours(self.workIMG,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)

            # print(len(self.contour))
            # for i in range(len(contour)):
            #     cv.drawContours(self.srcIMG,Counter,i,255,8)
            self.centers, self.principal_angles = self.get_contour_centers_principal_angles(self.contour)

            print("Centers",self.centers)
            print("principal Angles",self.principal_angles)

            self.render_result()

    def regionGrowing(self,k,j,i):
        # k: ROW
        # j: COL
        # m: ROW number
        # n: COL number
        # i: pixel
        #init parameters
        points = []
        m = self.workIMG.shape[0]
        n = self.workIMG.shape[1]

        #step 1
        self.workIMG[k,j] = i
        points.append([k,j])
        points.append([0,0])
        
        #step 2
        #Kernel Right
        if (j<n) and self.workIMG[k,j+1] == 255:
            self.workIMG[k,j+1] = i
            points.append([k,j+1])

        #step 3
        #Kernel Up
        if (k>1) and self.workIMG[k-1,j] == 255:
            self.workIMG[k-1,j] = i
            points.append([k-1,j])

        #step 4
        #Kernel Left
        if (j>1) and self.workIMG[k,j-1] == 255:
            self.workIMG[k,j-1] = i
            points.append([k,j-1])

        #step 5
        #Kernel Down
        if (k<m) and self.workIMG[k+1,j] == 255:
            self.workIMG[k+1,j] = i
            points.append([k+1,j])

        #Step 6
        while(True):
            # print(len(points))
            k,j = points.pop()
            if(k == 0 and j == 0):
                points.pop()
                break
            #step 2
            #Kernel Right
            if (j<n) and self.workIMG[k,j+1] == 255:
                self.workIMG[k,j+1] = i
                points.append([k,j+1])

            #step 3
            #Kernel Up
            if (k>1) and self.workIMG[k-1,j] == 255:
                self.workIMG[k-1,j] = i
                points.append([k-1,j])

            #step 4
            #Kernel Left
            if (j>1) and self.workIMG[k,j-1] == 255:
                self.workIMG[k,j-1] = i
                points.append([k,j-1])

            #step 5
            #Kernel Down
            if (k<m) and self.workIMG[k+1,j] == 255:
                self.workIMG[k+1,j] = i
                points.append([k+1,j])

    def get_contour_centers_principal_angles(self,contours):
        """
        Calculate the centers of the contours
        :param contours: Contours detected with find_contours
        :return: object centers as numpy array
        """

        if len(contours) == 0:
            return np.array([])

        # ((x, y), radius) = cv2.minEnclosingCircle(c)
        centers = np.zeros((len(contours), 2), dtype=np.int16)
        principal_angles = np.zeros((len(contours),1), dtype=np.double)
        self.M = []
        for i, c in enumerate(contours):
            self.M.append(cv.moments(c))
            # print(M)
            center = int(self.M[i]["m10"] / self.M[i]["m00"]), int(self.M[i]["m01"] / self.M[i]["m00"])
            principal_angles[i] = (math.atan2(2*self.M[i]['mu11'], (self.M[i]['mu02']-self.M[i]['mu20'])/2))*180/math.pi
            centers[i] = center
        return centers,principal_angles

    def render_result(self,line_length = 100):

        for i in range(len(self.contour)):
            cv.drawContours(self.resultIMG,self.contour,i,(255,0,255),5)
            cv.circle(self.resultIMG,(self.centers[i][0],self.centers[i][1]),2,(255,0,0),8)
            # print(self.centers[i][0])
            cv.line(self.resultIMG,
            (self.centers[i][0] -line_length , self.centers[i][1] - int(line_length * math.tan(math.atan2(2*self.M[i]['mu11'], self.M[i]['mu20']-self.M[i]['mu02'])/2))),
            (self.centers[i][0] +line_length , self.centers[i][1] + int(line_length * math.tan(math.atan2(2*self.M[i]['mu11'], self.M[i]['mu20']-self.M[i]['mu02'])/2))),
            (255,125,64),2)
            # cv.line(self.resultIMG,(100,200),(200,300),(255,255,0),10)
        for row in range(self.workIMG.shape[0]):
            for col in range(self.workIMG.shape[1]):
                self.workIMG[row,col] = self.workIMG[row,col] * 50
        
    def visulaize(self,*mats):
            for mat in mats:    
                if id(mat) == id(self.srcIMG):
                    cv.imshow("SRC",self.srcIMG)
                if id(mat) == id(self.workIMG):
                    cv.imshow("WORK",self.workIMG)
                if id(mat) == id(self.resultIMG):
                    cv.imshow("Result",self.resultIMG)
            if(len(mats) > 0):
                cv.waitKey(0)


if(len(sys.argv) != 2):
    print("python ./hm_3_b.py [image path]")
    quit(-1)

# filename = os.path.abspath("/home/robot/Desktop/HW/images/er7-4.jpg")
    
filename = os.path.abspath(sys.argv[1])

print("Reading File: " + filename)

#Init Image Processing Class for Homework 3-> Part B
ip = ImageProcessing(filename)
#call main process method
ip.Processing()
#visulaize result

ip.visulaize(ip.workIMG,ip.srcIMG,ip.resultIMG)

