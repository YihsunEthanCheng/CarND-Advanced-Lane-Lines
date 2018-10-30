# Advanced Lane Finding Project
---
## Objective
Create a image processing pipeline which does the following to extract/label the lane on highway
* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.
---
[//]: # (Image References)

[image0]: ./output_images/undistort_calibration1.jpg "Undistorted chessboard"
[image1]: ./output_images/undistort_test4.jpg "Undistorted scene"
[image2]: ./output_images/colorMask_test4.jpg "color mask"
[image3]: ./output_images/xGradMask_test4.jpg "Gradient mask"
[image4]: ./output_images/xGradColorMask_test4.jpg "Color/grdient combined mask" 
[image5]: ./output_images/warped_straight_lines1.jpg "Road Transformed"
[image6]: ./output_images/binWarped_test4.jpg  "Transfromed mask"
[image7]: ./output_images/polyfit_test4.jpg "Poly fit via window search"
[image8]: ./output_images/refinePoly_test4.jpg "Refine poly fit" 
[image9]: ./output_images/zone_test4.jpg "Unwarped lane zone"
[image10]: ./output_images/out_test4.jpg  "Output"
[video1]: ./output_images/labeled_project_video.mp4 "Video"

---
## How to run the project

1. "LaneFinder" is an image processing/recognition pipeline implemented as a python class in **"modules/laneProcPipe.py"**
2. The project can be executed by running the entry file "run_lane_proicess_pipeline.py"
3. The labeled video and intermediate result at each stage are located "output_images/" folder
---
## Camera Calibration

Camera calibration requires the following steps
1. Acquire a few checkerboard image placed at different distance with multiple tilting orientations. The images for this project are provided in 'camera_cl/' folder.
2. Compose "object points" to describe the corners appearance in each calibration image.
3. Run "findChessboardCorners" in opencv to find the corners as "image points" to meet the chessboard speicifications by the "object points"
4. Use the pairs of "object points" and "image points" to solve camera extrinsic matrix through LSE solutions and subsequently find the camera intrinsic matrix, which parameterizes the camera distrotion in 5 floating points numbers.
5. Below shows the result after camera calibration is done correctly.
![alt text][image0]
---
## Pipeline Descriptions (on Single Image)

### 1. Pipeline parameters

The parameters below are selected for the pipeline, which is organized as a python dict in  "run_lane_proicess_pipeline.py".

**params** = { </br>
        'calibPath' : 'camera_cal',</br>
        'imgPath'   : 'test_images',</br>
        'outPath'   : 'output_images',</br>
        'HLS_select' : 2, # pick saturation</br>
        'HLS_cut'  : [90, 255],</br>
        'gradx_cut': [20, 100],</br>
        'rows': 720,</br>
        'cols': 1280,</br>
        # peak search params       </br>
        'nwindows': 9,</br>
        'margin': 100, # width of the windows</br>
        'minpix': 150,</br>
        # Define conversions in x and y from pixels space to meters</br>
        'ym_per_pix': 30/720.,  # meters per pixel in y dimension</br>
        'xm_per_pix': 3.7/700, # meters per pixel in x dimension</br>
        'warp_xL': 320</br>
        } </br>

### 2. Undistortion

Use the camera matrix derived from camera calibration to undistort each input frame. The difference in natural image may not be easily seen. Refers to the image above to visualize the difference before and after undistortion.

![alt text][image1]

### 3. Color feature extraction

The input image is first transformed to HLS color space. As color saturation is less sensitive to lighting condition, the "S" channle is selected and thresholded as suggested in the parameter dictionary above.

![alt text][image2]

### 4. Gradient feature extraction

As we are searching for near-vertical lines, the gradeint change in the x-direction stands out as a lane discriminator. The feature is extract by Sobel filter with the threshold suggested in the parameter file as \[20,100\]. Below shows an example.

![alt text][image3]

### 5. Combined feature mask

Gradient/color features are often complimentary. Combined the two gives a much better look of the lanes as shown below.

![alt text][image4]


### 6. Perspective transfromation

Lanes are perspective trasnfromed in 2D images. To have a better view of the lane for extraction, a reverse perspective transformation called warping will do the trick. The problem is how to acquire the trasnformation matrix.  
To do this, we need straight line images as a groundtruth to derive the transformation. The two stright lines images in "test_images" folder serve the purpose. After many round of tuning, I have manully derive the source/destination window for perspective transformation as below. The two sets of points allow me to find the warper matrix.

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 200.,   719.  | 320., 719.    | 
| 580.5,  460.  | 320.,   0.    |
| 698.5,  460.  | 959.,   0.    |
| 1079.,  719.  | 959., 719.    |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image5]

The test image after warping appears to look as,

![alt text][image6]

### 7. Find lane pixels through window search (1st polynomial fit)

Two 200-pixel wide windows are placed in two halves (left/right) of the images hose inital locations are found through peak histogram search. From there, the window will be moved left/right based on the mean lane pixels locations in the urrent windows.
After lane pixels are found, a 2nd polynomial fit to lines in both side describes how roughly they are located, which created two searching areas for refine search in the next stage.

![alt text][image7]

### 8. Refine polynomial fit

Based on the windowed-search in the previous stage, the missing lane pixels can be better include in the resultant polynomial search zone. This zone are re-fit by another 2nd order ploynomial. And the result is fantasic as shown below.

![alt text][image8]


### 9. Unwarp painted lane zone back to the scene

This stage use the reverse perspective transfromation to unwarp the painted zone back to the scene in the input image as below.

![alt text][image9]


### 10. Calculate curvature and deviation to the center of the lane

**Curvature** are computed based the well-defined [**Radius of Curvature**](https://www.intmath.com/applications-differentiation/8-radius-curvature.php) metric. This computation is done in [landFinder.measure_curvature_real()](./modules/landProcPipe.py).

**Lane deviation** is found by the difference between the center of lane and the center of the x-axis at the bottom of the image. The computation is implemented in [landFinder.measure_center_deviation()](./modules/landProcPipe.py).

Both measurements are computed in pixels then converted to metric using the provided pixel to real scene conversion, also listed in the parameter struct above. 

### 11. Overlay lane zone and text on input image

Finally, the painted lane zone and the computed lane parameters are superpose on top of the input image to complete the pipeline processing.

![alt text][image10]

---

## Pipeline (video)

Here's a [link to my video result](./labeled_project_video.mp4)

---

## Discussion

1. The first problem I found is in the window search of lane pixels. The suggested threshold is 50 which I found is too low and resulting in shifting the windows by noise. This is often seen while dealing with dash lane lines.  I increased the parameter "minpix" to 150 and the problme wemt away.

2. My lane finder pipeline apparently failed on the two **challenge videos**.  There are a few caused that I observed as explained below.

**low light condition**: In low light situation, the saturation and gradient are both weakened.
**Shading**: Stuctured shading such as buildings or bridges often create gradient features to confuse the gradient extraction.
**Vertical texture on road**: very often, highways are constructed by long concrete plates which create vertical edges to confuse the edge extractor. 
**Narrow lanes in mountain area**: strutures along road lanes such as trenches are also creating edges parallel to road lanes and fails window search.
**Combinations of all**: any combination of the above features in the scene further aggrevate the false positive errors.

3. The solution to all the problems in the challenge videos require multple agent. Below lists the agents whom can be combined to fix majority of the problem.

* Convolutional features: such as matched filter can be used to boost the signal of lane lines as they appear in certain color and certain shape.
* Color space: Saturation channel along won't be enough.  We might combined with the Red channel in RGB color space to imrpove the color selection robustness.
* Kalman filters: lane lines are predictable in the temporal space. As soon as we identify corret lanes, we can begin to track the lane more closely using the speed of the vehicle to predict the lane line locations in the next frame. This will help elimiate a lot of false positives.




