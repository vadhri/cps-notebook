Build
--- 
For ease of metrics collection, command line arguments are used for providing detector descriptor combination.
Example usage:  `./3D_object_tracking FAST ORB`

FP.1 Match 3D Objects
---------------------

The `matchBoundingBoxes()` function identifies and filters matching bounding boxes between two frames based on keypoint matches.
First, it checks if each keypoint in the current and previous frames lies within any bounding box, associating the keypoints 
with the corresponding box IDs. It then groups these matches by bounding box pairs and filters them using the Euclidean distance
between keypoints, retaining only those within a calculated threshold. Finally, for each bounding box in the previous frame, the
function determines the best match in the current frame by selecting the box pair with the highest count of filtered matches and
stores these in `bestMatches`.

FP.2 Compute Lidar-based TTC
----------------------------
Compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.

The `computeTTCLidar` function is modified to calculate the Time-to-Collision (TTC) using Lidar data from two consecutive frames.

### 1. **Sorting Lidar Points:**
   - The function begins by sorting the Lidar points from both the previous and current frames based on their distance from the sensor in the forward direction.
   - This is done to identify the closest Lidar points, which are critical for accurate TTC calculation.

### 2. **Identifying Key Distances:**
   - After sorting, the function selects the median distance of the Lidar points from both the current and previous frames. The median is used because it is less sensitive to noise or outliers than the minimum or maximum distance.
   - The median distance in the current frame represents how close the object is now, while the median distance in the previous frame represents how close the object was in the last frame.

### 3. **Calculating Time-to-Collision (TTC):**
   - The TTC is calculated using the formula:
     \[
     TTC = \frac{d1 \times dt}{d0 - d1}
     \]
   - Here, `d1` is the median distance from the current frame, `d0` is the median distance from the previous frame, and `dt` is the time difference between the frames (calculated as the inverse of the frame rate).
   - This formula estimates the time it will take for the object to collide with the sensor if it continues moving at its current speed.

### 4. **Purpose:**
   - The TTC provides an estimate of how much time is left before a collision occurs based on the current distance and speed of the object.
   
FP.3 Associate Keypoint Correspondences with Bounding Boxes
--------------------------------------------------------

The clusterKptMatchesWithROI function clusters keypoint matches within a bounding box's region of interest (ROI).
It starts by calculating the Euclidean distances between matched keypoints from the previous and current frames, considering only keypoints that lie within the ROI.

The function then computes the mean of these distances.

In the next step, it filters keypoint matches based on their distance, retaining only those whose distances are less than 1.3 times the computed mean.

The filtered keypoints and their corresponding matches are then added to the bounding box's keypoints and matches lists.
This process helps identify reliable keypoint matches within the ROI.

FP.4 Compute Camera-based TTC
------------------------------

The `computeTTCCamera` function calculates the Time-to-Collision (TTC) using keypoint matches between consecutive camera frames. This method estimates how long it will take for an object to collide with the camera based on the change in distance between matched keypoints over time.

### 1. **Initialize Variables:**
   - The function begins by initializing a vector to store the ratios of distances between matched keypoints in the current and previous frames (`distanceRatios`).
   - A threshold (`minDistanceThreshold`) is set to ignore very small distances that may result in inaccurate ratios.

### 2. **Iterate Over Keypoint Matches:**
   - The function uses nested loops to iterate over all possible pairs of keypoint matches between the current and previous frames. Each keypoint match corresponds to a point in the previous frame and its corresponding point in the current frame.

### 3. **Compute Distances Between Keypoints:**
   - For each pair of keypoint matches, the function calculates the Euclidean distance between the keypoints in both the current and previous frames.
   - The `distCurr` variable represents the distance between two keypoints in the current frame, while `distPrev` represents the distance between the corresponding keypoints in the previous frame.

### 4. **Calculate Distance Ratios:**
   - If the distance between the keypoints in the previous frame (`distPrev`) is greater than a small epsilon value (to avoid division by zero), and the distance in the current frame (`distCurr`) is above the threshold, the ratio of `distCurr` to `distPrev` is calculated.
   - This ratio indicates how much the distance between the keypoints has changed from the previous frame to the current frame. These ratios are stored in the `distanceRatios` vector.

### 5. **Compute the Median Distance Ratio:**
   - After all valid distance ratios have been computed, the function sorts the `distanceRatios` vector to find the median value. The median is less affected by outliers and noise than the mean, making it a more robust measure for this purpose.

### 6. **Calculate TTC:**
   - The function calculates the Time-to-Collision using the formula:
     \[
     TTC = -\frac{dT}{1 - \text{median}}
     \]
     where `dT` is the time between frames (calculated as the inverse of the frame rate).
   - This formula estimates the time it will take for the object to collide with the camera if it continues moving at its current speed.

### 7. **Output TTC:**
   - The calculated TTC is stored in the reference variable `TTC`, which can be used later in the program.

FP.5 Performance Evaluation 1
-----------------------------
Find examples where the TTC estimate of the Lidar sensor does not seem plausible. Describe your observations and provide a sound argumentation why you think this happened.

In the TTC estimates provided by the LiDAR sensor, certain frames exhibit implausibly short or long TTC values, such as when the TTC suddenly drops between frames 12-16, indicating a rapid deceleration that the model fails to accurately capture. This discrepancy arises due to the Broken Velocity Model, which assumes constant or linearly changing velocities and does not account for obstacles that decelerate non-linearly. 

As a result, when an obstacle slows down more rapidly than the model predicts, the TTC becomes unreliable, either underestimating or overestimating the actual time to collision. These inconsistencies highlight the model’s inability to handle real-world scenarios where obstacles frequently decelerate unpredictably. To improve accuracy, a more robust model that considers varying deceleration patterns should be employed.

Examples are provided in the path : ./build/images_shitomasi_brief

FP.6 Performance Evaluation 2
------------------------------

The three fast and reliable detectors are SHITOMASI, FAST and AKAZE as per the data i collected. 
The excel sheet has the tabular format and the raw data is in the build folder. 





