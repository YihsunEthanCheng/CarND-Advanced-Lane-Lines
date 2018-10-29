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
class laneFinder(object):
    
    
    def __init__(self, params):
        self.__dict__= params.copy()
        try:
            (self.mtx, self.dist, self.Tp) = pickle.load(open('camera_params.pickle','rb'))
        except:
            self.calib()
        self.ploty = np.linspace(0, self.rows-1, self.rows)

        
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
    
    
    def unit_test(self):
        # dump undistortion results
        
        # dump perspective warping         
        files = glob.glob(self.imgPath + '/*.jpg')
        for i, file in enumerate(files):
            self.warp_perspective(self.undistort(mpimg.imread(file)), i+1)
            fig = plt.figure(i+1)
            plt.show()
            fig.savefig(self.outPath + '/warped_' + os.path.basename(file))
        
        # dump pipeline intermediate results
    
    def find_polylanes(self,binary_warped, figID = 0):
        # Take a histogram of the bottom half of the image
        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
        out_img = []
        if figID >0:
            plt.figure(figID)
            plt.clf()
            # Create an output image to draw on and visualize the result
            out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int(histogram.shape[0]//2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        # Set height of windows - based on nwindows above and image shape
        window_height = np.int(binary_warped.shape[0]//self.nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = binary_warped.nonzero()
        nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])
        # stashed away for refine stage
        self.warped_nonzeroy, self.warped_nonzerox = nonzeroy, nonzerox
        # Current positions to be updated later for each window in nwindows
        leftx_current = leftx_base
        rightx_current = rightx_base
        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []
        # Step through the windows one by one
        for window in range(self.nwindows):
            # select window veritical range
            win_y_low = binary_warped.shape[0] - (window+1)*window_height
            win_y_high = binary_warped.shape[0] - window*window_height
            # Select window horizontal range
            win_xleft_low = leftx_current - self.margin 
            win_xleft_high = leftx_current + self.margin 
            win_xright_low = rightx_current - self.margin
            win_xright_high = rightx_current + self.margin 
            if figID > 0:
                # Draw the windows on the visualization image
                cv2.rectangle(out_img,(win_xleft_low,win_y_low),
                (win_xleft_high,win_y_high),(0,255,0), 2) 
                cv2.rectangle(out_img,(win_xright_low,win_y_low),
                (win_xright_high,win_y_high),(0,255,0), 2) 
                plt.imshow(out_img)
            #Identify the nonzero pixels in x and y within the window
            good_left_inds  = ((nonzerox > win_xleft_low) & (nonzerox <= win_xleft_high) & \
                (nonzeroy > win_y_low) & (nonzeroy <= win_y_high)).nonzero()[0]
            good_right_inds = ((nonzerox > win_xright_low) & (nonzerox <= win_xright_high) & \
                (nonzeroy > win_y_low) & (nonzeroy <= win_y_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            # found > minpix pixels, recenter next window
            # (`right` or `leftx_current`) on their mean position 
            if len(good_left_inds) > self.minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > self.minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))
        # Concatenate the arrays of indices (previously was a list of lists of pixels)
        try:
            left_lane_inds = np.concatenate(left_lane_inds)
            right_lane_inds = np.concatenate(right_lane_inds)
        except ValueError:
            # Avoids an error if the above is not implemented fully
            pass
        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
    
        # Fit a second order polynomial to each using `np.polyfit`
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        if figID > 0:
            # Generate x and y values for plotting
            try:
                left_fitx = left_fit[0]*self.ploty**2 + left_fit[1]*self.ploty + left_fit[2]
                right_fitx = right_fit[0]*self.ploty**2 + right_fit[1]*self.ploty + right_fit[2]
            except TypeError:
                # Avoids an error if `left` and `right_fit` are still none or incorrect
                print('The function failed to fit a line!')
                left_fitx = 1*self.ploty**2 + 1*self.ploty
                right_fitx = 1*self.ploty**2 + 1*self.ploty
            # Colors in left/right lanes
            out_img[lefty, leftx] = [255, 0, 0]
            out_img[righty, rightx] = [0, 0, 255]
            # Plots the left and right polynomials on the lane lines
            plt.plot(left_fitx, self.ploty, color='yellow')
            plt.plot(right_fitx, self.ploty, color='yellow')
            plt.imshow(out_img)
            plt.title('Lanes pixels and Windows')
        return left_fit, right_fit    
            

    def points2polyx(self, x, y):
        coef = np.polyfit(y, x, 2)
        return coef[0]*self.ploty**2 + coef[1]*self.ploty + coef[2], coef


    def paintBetweenLines(self, window_img, line_fitx_left, line_fitx_right, color):
        '''
        Paint a region between two lines of x-coordinates 
        '''
        # form polygon
        line_left  = np.vstack([line_fitx_left, self.ploty])
        line_right = np.vstack([line_fitx_right, self.ploty])[:,::-1]
        polygon = np.hstack((line_left, line_right)).T
        cv2.fillPoly(window_img, np.int_([polygon]), color)
        return window_img


    def refine_polylanes(self, binary_warped, left_fit, right_fit, figID = 0):
        '''
        Refines the polynomial fits using the prvious coarse estimate
        '''
        # Grab activated pixels
        nonzeroy, nonzerox = self.warped_nonzeroy, self.warped_nonzerox
        # find lane pixels within previous fond polynomial
        left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + 
            left_fit[1]*nonzeroy + left_fit[2] - self.margin)) &
            (nonzerox < (left_fit[0]*(nonzeroy**2) + 
            left_fit[1]*nonzeroy + left_fit[2] + self.margin)))
        right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + 
            right_fit[1]*nonzeroy + right_fit[2] - self.margin)) &
            (nonzerox < (right_fit[0]*(nonzeroy**2) + 
            right_fit[1]*nonzeroy + right_fit[2] + self.margin)))
        # Again, extract left and right line pixel positions
        leftx, lefty = nonzerox[left_lane_inds], nonzeroy[left_lane_inds] 
        rightx, righty = nonzerox[right_lane_inds], nonzeroy[right_lane_inds]
        # refit new polynomials
        left_fitx, left_fit_refined = self.points2polyx(leftx, lefty)
        right_fitx, right_fit_refined = self.points2polyx(rightx, righty)
        # plot new polyfit on both lines
        if figID > 0:
            out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
            out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
            # overlay
            overlay = self.paintBetweenLines(np.zeros_like(out_img), 
                left_fitx-self.margin, left_fitx+self.margin, (0,255,0))
            overlay = self.paintBetweenLines(overlay, 
                right_fitx-self.margin, right_fitx+self.margin, (0,255,0))
            result = cv2.addWeighted(out_img, 1, overlay, 0.3, 0)
            plt.figure(figID)
            plt.clf()
            plt.imshow(result)
            plt.plot(left_fitx, self.ploty, color='yellow')
            plt.plot(right_fitx, self.ploty, color='yellow')
        return left_fitx, right_fitx, left_fit_refined, right_fit_refined
        
    def measure_curvature_real(self, left_fit, right_fit):
        '''
        Calculates the curvature of polynomial functions in meters.
        '''
        y_eval = np.max(self.ploty)
        left_curverad = ((1 + (2*left_fit[0]*y_eval*self.ym_per_pix+left_fit[1])**2)**(1.5))/abs(2*left_fit[0])
        right_curverad = ((1 + (2*right_fit[0]*y_eval*self.ym_per_pix+right_fit[1])**2)**(1.5))/abs(2*right_fit[0])
        return left_curverad, right_curverad

    def measure_center_deviation(self, left_fit, right_fit):
        y_eval = np.max(self.ploty) 
        xBottomLeft = left_fit[0] * y_eval**2 + left_fit[1] * y_eval +  left_fit[2]
        xBottomRight = right_fit[0] * y_eval**2 + right_fit[1] * y_eval +  right_fit[2]
        return (self.cols/2.0 - (xBottomLeft + xBottomRight)/2.0)*self.xm_per_pix
        
    def output_message(self, left_fit, right_fit):
        curvature = int(np.mean(self.measure_curvature_real(left_fit, right_fit)))
        xdif = self.measure_center_deviation(left_fit, right_fit)
        return 'Radius of Curvature = {:d}m\nVechicle is {:4.2f}(m) {} of center'.\
            format(curvature, abs(xdif), 'left' if xdif < 0 else 'right') 


#%%
    def process(self, img, show = False):
        # step 1: undistortion
        undist = self.undistort(img, int(show)*1)
        # step 2: extract color feature
        color_mask = self.select_color(undist, int(show)*2, 'color mask')
        #step 3: extract x-gradient feature
        grad_mask = self.extract_gradient(undist, int(show)*3, 'x-gradient mask')
        # step 4: combine color + gradient  
        combo = (color_mask | grad_mask)
        if show:
            stack_mask = np.dstack((np.zeros_like(color_mask), grad_mask, color_mask))*255
            plt.figure(int(show)*4)
            plt.imshow(stack_mask)
            plt.title('Combined color + gradient mask')        

        return self.warp_perspective(combo)
        # step 5: apply perspective transformation
        
        # step 6: extract lanes
        
        # step 7: fit curves
        
        # step 8: reverse warping back to color image
        
        
        # step 8: paint lanes
        
        
        
        return img
    

