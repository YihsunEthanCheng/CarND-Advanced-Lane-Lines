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
import os
#%%
class laneProcessPipeline(object):
    
    
    def __init__(self, params):
        self.__dict__= params.copy()
        try:
            (self.mtx, self.dist, self.Tp) = pickle.load(open('camera_params.pickle','rb'))
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
            img = mpimg.imread(fn)
            gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
           
            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, (9,6),None)

            # If found, add object points, image points
            if ret == True:
                objpoints.append(objp)
                imgpoints.append(corners)
        
                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (9,6), corners, ret)
        ret, self.mtx, self.dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        self.find_perspective_transform()
        pickle.dump((self.mtx, self.dist, self.Tp),open('camera_params.pickle','wb'))
        
    def find_perspective_transform(self):
        src = np.array([[[200, 720]], [[568, 468]], [[712, 468]], [[1080,720]]], dtype= np.float32)
        dst = np.array([[[300, 720]], [[300, 0]], [[980, 0]], [[980,720]]], dtype= np.float32)
        self.Tp = cv2.getPerspectiveTransform(src, dst)        
       
    def warp_perspective(self, img, figID = 0):
        wimg = cv2.warpPerspective(img, self.Tp, img.shape[:2][::-1], flags=cv2.INTER_LINEAR)
        if figID > 0:
            wimg[:,290:300,0] = 255
            wimg[:,980:990,0] = 255
            fig = plt.figure(figID)
            fig.clf()
            axes = fig.subplots(1,2, sharey = True)
            axes[0].imshow(img)
            axes[0].set_title('original')
            axes[1].imshow(wimg)
            axes[1].set_title('Warped')
            fig.subplots_adjust(left = 1/16., right = 1.0 - 1./16,
                top = 1- 1./32, bottom = 1./32 , wspace=1./32 )
        return wimg
    
    
    def cut_and_show(self, gray, cut, figID, title):
        mask = np.zeros_like(gray)
        mask[(gray >= cut[0]) & (gray <= cut[1])] = 1
        if figID > 0:
            plt.figure(figID)
            plt.imshow(mask*255, cmap = 'gray')
            plt.title(title)
        return mask   
            
    
    def undistort(self, img, figID = 0):
        undist = cv2.undistort(img, self.mtx, self.dist, None, self.mtx)
        if figID > 0:
            plt.figure(figID)
            plt.imshow(np.hstack((img, undist)))
            plt.title('Input image   vs.   undistorted image')
        return undist
        
    
    def imread(self, fname):
        return mpimg.imread(self.imgPath + '/' + fname)
        
    
    def select_color(self, img, figID = 0, title = ''):
        hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)[:,:,self.HLS_select];
        return self.cut_and_show(hls, self.HLS_cut, figID, title)
        
    
    def extract_gradient(self, img, figID = 0, title = ''):
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        gradx = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
        abs_sobelx = np.absolute(gradx)
        scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
        return self.cut_and_show(scaled_sobel, self.gradx_cut, figID, title)
    
    
    def test_perspetive_transform(self):
        files = glob.glob(self.imgPath + '/*.jpg')
        for i, file in enumerate(files):
            self.warp_perspective(self.undistort(mpimg.imread(file)), i+1)
            fig = plt.figure(i+1)
            plt.show()
            fig.savefig(self.outPath + '/warped_' + os.path.basename(file))
    
    
    def process(self, img, show = False):
        # step 1: undistortion
        undist = self.undistort(img, int(show)*1)
        # step 2: extract color feature
        color_mask = self.select_color(undist, int(show)*2, 'color mask')
        #step 3: extract x-gradient feature
        grad_mask = self.extract_gradient(undist, int(show)*3, 'x-gradient mask')
        # step 4: combine color + gradient  
        stack_mask = np.dstack((np.zeros_like(color_mask), grad_mask, color_mask))*255
        if show:
            plt.figure(int(show)*4)
            plt.imshow(stack_mask)
            plt.title('Combined color + gradient mask')         
        # step 5: apply perspective transformation
        
        # step 6: extract lanes
        
        # step 7: fit curves
        
        # step 8: reverse warping back to color image
        
        
        # step 8: paint lanes
        
        
        
        return img
    

