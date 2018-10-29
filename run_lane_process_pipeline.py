#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 28 00:17:06 2018

@author: Ethan Cheng
"""
%load_ext autoreload
%autoreload 2

from modules.laneProcPipe import laneFinder

params = {
        'calibPath' : 'camera_cal',
        'imgPath'   : 'test_images',
        'outPath'   : 'output_images',
        'HLS_select' : 2, # pick saturation
        'HLS_cut'  : [90, 255],
        'gradx_cut': [20, 100],
        'rows': 720,
        'cols': 1280,
        # peak search params       
        'nwindows': 9,
        'margin': 100, # width of the windows
        'minpix': 150,
        # Define conversions in x and y from pixels space to meters
        'ym_per_pix': 30/720.,  # meters per pixel in y dimension
        'xm_per_pix': 3.7/700, # meters per pixel in x dimension
        'warp_xL': 320
        } 
        
lane_detect = laneFinder(params)
lane_detect.calib()
lane_detect.dump_pipeline_staged_result()
