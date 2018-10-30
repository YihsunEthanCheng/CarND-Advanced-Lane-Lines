## Advanced Lane Finding Project
---
### Objective
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

[image1]: ./output_images/undistort_calibration1.jpg "Undistorted"
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
### HOW TO RUN the PROJECT AND PROJECT FOLDERS
1. "LaneFinder" is an image processing/recognition pipeline implemented as a python class in **"modules/laneProcPipe.py"**
2. The project can be executed by running the entry file "run_lane_proicess_pipeline.py"
3. The labeled video and intermediate result at each stage are located "output_images/" folder
---
### Camera Calibration

#### Camera calibration requires the following steps
1. Acquire a few checkerboard image placed at different distance with multiple tilting orientations. The images for this project are provided in 'camera_cl/' folder.
2. Compose "object points" to describe the corners appearance in each calibration image.
3. Run "findChessboardCorners" in opencv to find the corners as "image points" to meet the chessboard speicifications by the "object points"
4. Use the pairs of "object points" and "image points" to solve camera extrinsic matrix through LSE solutions and subsequently find the camera intrinsic matrix, which parameterizes the camera distrotion in 5 floating points numbers.
5. Below shows the result after camera calibration is done correctly.

![alt text][image1]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one:
![alt text][image2]

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

I used a combination of color and gradient thresholds to generate a binary image (thresholding steps at lines # through # in `another_file.py`).  Here's an example of my output for this step.  (note: this is not actually from one of the test images)

![alt text][image3]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform includes a function called `warper()`, which appears in lines 1 through 8 in the file `example.py` (output_images/examples/example.py) (or, for example, in the 3rd code cell of the IPython notebook).  The `warper()` function takes as inputs an image (`img`), as well as source (`src`) and destination (`dst`) points.  I chose the hardcode the source and destination points in the following manner:

```python
src = np.float32(
    [[(img_size[0] / 2) - 55, img_size[1] / 2 + 100],
    [((img_size[0] / 6) - 10), img_size[1]],
    [(img_size[0] * 5 / 6) + 60, img_size[1]],
    [(img_size[0] / 2 + 55), img_size[1] / 2 + 100]])
dst = np.float32(
    [[(img_size[0] / 4), 0],
    [(img_size[0] / 4), img_size[1]],
    [(img_size[0] * 3 / 4), img_size[1]],
    [(img_size[0] * 3 / 4), 0]])
```

This resulted in the following source and destination points:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 585, 460      | 320, 0        | 
| 203, 720      | 320, 720      |
| 1127, 720     | 960, 720      |
| 695, 460      | 960, 0        |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image4]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

Then I did some other stuff and fit my lane lines with a 2nd order polynomial kinda like this:

![alt text][image5]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

I did this in lines # through # in my code in `my_other_file.py`

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

I implemented this step in lines # through # in my code in `yet_another_file.py` in the function `map_lane()`.  Here is an example of my result on a test image:

![alt text][image6]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./project_video.mp4)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  
