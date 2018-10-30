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
            (self.mtx, self.dist, self.Tp, self.TpInv) = pickle.load(open('camera_params.pickle','rb'))
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
        pickle.dump((self.mtx, self.dist, self.Tp, self.TpInv),open('camera_params.pickle','wb'))
        
    def find_perspective_transform(self):
        endSpan = 59.5
        src = np.array([[[200, self.rows-1]], 
                        [[(self.cols-1)/2.0-endSpan, 460]], 
                        [[(self.cols-1)/2.0+endSpan, 460]], 
                        [[(self.cols-1)-200,self.rows-1]]], dtype= np.float32)
        xL, xR = self.warp_xL , self.cols - self.warp_xL -1
        dst = np.array([[[xL, self.rows-1]], [[xL, 0]], [[xR, 0]], [[xR,self.rows-1]]], dtype= np.float32)
        self.Tp = cv2.getPerspectiveTransform(src, dst)        
        self.TpInv = np.linalg.inv(self.Tp)

    def warp_perspective(self, img):
        return cv2.warpPerspective(img, self.Tp, img.shape[:2][::-1], flags=cv2.INTER_LINEAR)
    
    def unwarp_perspective(self,img):
        return cv2.warpPerspective(img, self.TpInv, img.shape[:2][::-1], flags=cv2.INTER_LINEAR)

    
    def threshold(self, gray, cut):
        mask = np.zeros_like(gray)
        mask[(gray >= cut[0]) & (gray <= cut[1])] = 1
        return mask   
            
    
    def undistort(self, img):
        return cv2.undistort(img, self.mtx, self.dist, None, self.mtx)
        
    
    def imread(self, fname):
        return mpimg.imread(self.imgPath + '/' + fname)
        
    
    def select_color(self, img):
        hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)[:,:,self.HLS_select];
        return self.threshold(hls, self.HLS_cut)
        
    
    def extract_gradient(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        gradx = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
        abs_sobelx = np.absolute(gradx)
        scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
        return self.threshold(scaled_sobel, self.gradx_cut)
    
      
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
            # Generate x and y values for plottinge
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
        return int((1+left_curverad + right_curverad)/2.0)


    def measure_center_deviation(self, left_fit, right_fit):
        y_eval = np.max(self.ploty) 
        xBottomLeft = left_fit[0] * y_eval**2 + left_fit[1] * y_eval +  left_fit[2]
        xBottomRight = right_fit[0] * y_eval**2 + right_fit[1] * y_eval +  right_fit[2]
        return (self.cols/2.0 - (xBottomLeft + xBottomRight)/2.0)*self.xm_per_pix


    def draw_overlay(self, img, unwarped_overlay, curvature, xdev):
        outText = ['Radius of Curvature = {:d}m'.format(curvature),
            'Vechicle is {:4.2f}(m) {} of center'.format(abs(xdev), \
            'left' if xdev < 0 else 'right') ]
        out_img = cv2.addWeighted(img, 1.0, unwarped_overlay, 0.3, 0)
        cv2.putText(out_img, outText[0], (50,60), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255),2)
        cv2.putText(out_img, outText[1], (50,120), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255),2)
        return out_img
    
#%%
    def process(self, img):
        # step 1: undistortion
        undist = self.undistort(img)
        # step 2: extract color feature
        color_mask = self.select_color(undist)
        #step 3: extract x-gradient feature
        grad_mask = self.extract_gradient(undist)
        # step 4: combine color + gradient  
        combo = (color_mask | grad_mask)
        # step 5: apply perspective transformation
        binary_warped = self.warp_perspective(combo)
        # step 6: find lines of a lane from polynomial fit
        left_fit, right_fit = self.find_polylanes(binary_warped)
        # step 7: refine polynomial fit with the guidanc of the previous polynomial fit
        left_fitx, right_fitx, left_fit_refined, right_fit_refined = \
            self.refine_polylanes(binary_warped, left_fit, right_fit)
        # step 8: generate overlay image from unwarping the painted lanes
        overlay = self.paintBetweenLines(np.zeros_like(img), left_fitx, right_fitx, (0,255,0))
        unwarped_overlay = self.unwarp_perspective(overlay)
        # step 9: compute output varaibles in curvature/center deviation
        xdev = self.measure_center_deviation(left_fit, right_fit)
        curvature = self.measure_curvature_real(left_fit, right_fit)
        # step 10: overlay painted lanes and output text on top of the imput image
        return self.draw_overlay(undist,unwarped_overlay, curvature, xdev)


    def dump_pipeline_staged_result(self, fn = ''):
        if len(fn) == 0:        
            # dump perspective warping for all test image       
            files =np.array(glob.glob(self.imgPath + '/*.jpg'))
            for i, file in enumerate(files):
                img = self.undistort(mpimg.imread(file))
                wimg = self.warp_perspective(img)
                xL, xR = self.warp_xL , self.cols - self.warp_xL
                # paint vertical line for diff
                wimg[:,xL-8:xL] = (255,0,0)
                wimg[:,xR:xR+8] = (255,0,0)
                fig = plt.figure()
                fig.clf()
                axes = fig.subplots(1,2, sharey = True)
                axes[0].imshow(img)
                axes[0].set_title('original')
                axes[1].imshow(wimg)
                axes[1].set_title('Warped')
                fig.subplots_adjust(left = 1/16., right = 1.0 - 1./16,
                    top = 1- 1./32, bottom = 1./32 , wspace=1./32 )
                fig.savefig(self.outPath + '/warped_' + os.path.basename(file))
                return
        
        # dump pipeline intermediate results
        # step 1: camera calibration dump undistortion results
        imgs = [ mpimg.imread('camera_cal/calibration1.jpg'), self.imread(fn)]
        fns = ['calibration1.jpg', fn]
        for fn_i, img in zip(fns, imgs): 
            undist = self.undistort(img)
            fig = plt.figure()
            axes = fig.subplots(1,2, sharey = True)
            axes[0].imshow(img)
            axes[0].set_title('Raw Input')
            axes[1].imshow(undist)
            axes[1].set_title('undistorted')
            fig.subplots_adjust(left = 1/16., right = 1.0 - 1./16,
                    top = 1- 1./32, bottom = 1./32 , wspace=1./32 )
            fig.savefig(self.outPath + '/undistort_' + fn_i)
                    
        # step 2: extract color feature
        img = self.imread(fn)
        undist = self.undistort(img)
        color_mask = self.select_color(undist)
        fig = plt.figure()
        plt.imshow(color_mask*255, cmap = 'gray')
        plt.title('color mask')
        fig.savefig(self.outPath + '/colorMask_' + os.path.basename(fn))
        
        #step 3: extract x-gradient feature
        grad_mask = self.extract_gradient(undist)
        fig = plt.figure()
        plt.imshow(color_mask*255, cmap = 'gray')
        plt.title('x-gradient mask')
        fig.savefig(self.outPath + '/xGradMask_' + os.path.basename(fn))
        
        # step 4: combine color + gradient  
        combo = (color_mask | grad_mask)
        fig = plt.figure()
        plt.imshow(np.dstack((np.zeros_like(color_mask), grad_mask, color_mask))*255)
        plt.title('Combined color + gradient mask') 
        fig.savefig(self.outPath + '/xGradColorMask_' + os.path.basename(fn))
        
        # step 5: apply perspective transformation
        binary_warped = self.warp_perspective(combo)
        fig = plt.figure()
        plt.imshow(binary_warped*255)
        plt.title('binary warped mask') 
        fig.savefig(self.outPath + '/binWarped_' + os.path.basename(fn))
        
        # step 6: find lines of a lane from polynomial fit
        fig = plt.figure(1)
        fig.clf()
        left_fit, right_fit = self.find_polylanes(binary_warped, 1)
        plt.title('Windowed polynomial search') 
        fig.savefig(self.outPath + '/polyfit_' + os.path.basename(fn))
        
        # step 7: refine polynomial fit with the guidanc of the previous polynomial fit
        fig = plt.figure(2)
        fig.clf()
        left_fitx, right_fitx, left_fit_refined, right_fit_refined = \
            self.refine_polylanes(binary_warped, left_fit, right_fit, 2)
        plt.title('Refined polynomial fit') 
        fig.savefig(self.outPath + '/refinePoly_' + os.path.basename(fn))
        
        # step 8: generate overlay image from unwarping the painted lanes
        overlay = self.paintBetweenLines(np.zeros_like(undist), left_fitx, right_fitx, (0,255,0))
        unwarped_overlay = self.unwarp_perspective(overlay)
        fig = plt.figure()
        plt.imshow(unwarped_overlay)
        plt.title('Unwarped lane zone') 
        fig.savefig(self.outPath + '/zone_' + os.path.basename(fn))
        
        # step 9: compute output varaibles in curvature/center deviation
        xdev = self.measure_center_deviation(left_fit, right_fit)
        curvature = self.measure_curvature_real(left_fit, right_fit)
        
        # step 10: overlay painted lanes and output text on top of the imput image
        final = self.draw_overlay(undist,unwarped_overlay, curvature, xdev)
        fig = plt.figure()
        plt.imshow(final)
        plt.title('Pipeline output') 
        fig.savefig(self.outPath + '/out_' + os.path.basename(fn))
        
        
        