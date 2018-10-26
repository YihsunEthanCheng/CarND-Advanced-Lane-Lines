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


class laneProcessPipeline(object):
    
    def __init__(self, calibPath  = 'camera_cal', imgPath = '../test_images/'):
        self.calibPath = calibPath
        
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

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        img = cv2.imread(fns[0])
        dst = cv2.undistort(img, mtx, dist, None, mtx)
        sideBySide = np.hstack((img, dst))
        return sideBySide
    
    #%%
    def extract_edge():
        
    def apply_ROI()    
 
    def transform_color():
        
    def select_color():
        
    def select_gradient():
        
    def extract_gradeient():
        
    
#%%
        
lpp = laneProcessPipeline()    
img = lpp.calib()
