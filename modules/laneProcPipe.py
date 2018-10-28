#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 25 23:40:06 2018

@author: Ethan Cheng
"""

import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pickle

#%%
class laneProcessPipeline(object):
    
    def __init__(self, params):
        self.__dict__= params.copy()
        try:
            (self.mtx, self.dist) = pickle.load(open('camera_params.pickle','rb'))
        except:
            self.calib()
        img = cv2.imread(self.calibPath + '/calibration1.jpg')
        self.imgCalibOut = np.hstack((img, self.undistort(img)))

    def calib(self):
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((6*9,3), np.float32)
        objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
        fns = glob.glob(self.calibPath + '/calibration*.jpg')
        
        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d points in real world space
        imgpoints = [] # 2d points in image plane.
        for fn in fns:
            img = cv2.imread(fn)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
           
            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, (9,6),None)

            # If found, add object points, image points
            if ret == True:
                objpoints.append(objp)
                imgpoints.append(corners)
        
                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (9,6), corners, ret)

        ret, self.mtx, self.dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        pickle.dump((lpp.mtx, lpp.dist),open('camera_params.pickle','wb'))
        return self.imgCalibOut
        
    def cut_and_show(self, gray, cut, figID):
        mask = np.zeros_like(gray)
        mask[(gray >= cut[0]) & (gray <= cut[1])] = 1
        if figID > 0:
            plt.figure(figID)
            plt.imshow(mask*255, cmap = 'gray')
        return mask   
            
    def undistort(self, img):
        return cv2.undistort(img, self.mtx, self.dist, None, self.mtx)
        
    def process(self, img):
        return img
    
    def imread(self, fname):
        return cv2.imread(self.imgPath + '/' + fname)
        
    def select_color(self, img, figID = 0):
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)[:,:,self.HLS_select];
#        color_mask = np.zeros_like(gray)
#        color_mask[(gray > self.HLS_cut[0]) & (gray <= self.HLS_cut[1])] = 1
        return self.cut_and_show(hls, self.HLS_cut, figID)
        
    def extract_gradeient(self, img, figID = 0):
        img = mpimg.imread(self.imgPath + '/' + fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gradx = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
        abs_sobelx = np.absolute(gradx)
        scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
#        sxbinary = np.zeros_like(scaled_sobel)
#        sxbinary[(scaled_sobel >= gradx_cut[0]) & (scaled_sobel <= thresh_max)] = 1
        self.cut_and_show(scaled_sobel, self.gradx_cut, figID)

    
    #%%
    def extract_edge():
        
    def apply_ROI()    
 
        
    def select_color():
        
    def select_gradient():
         
    
#%%
params = {
        'calibPath' : 'camera_cal',
        'imgPath'   : 'test_images',
        'HLS_select' : 2, # pick saturation
        'HLS_cut' : [90, 255],
        'gradx_cut': [20, 100]
        } 
        
lpp = laneProcessPipeline(params)
#imshow(self.imgCalibOut)

#%%
img = lpp.imread('test1.jpg')
lpp.select_color(img, 2)







