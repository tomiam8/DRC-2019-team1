# DRC-2019-team1
Sorry for the mess of files (probably should have used git properly, opps) - the latest / final file is andre_vision.py (https://github.com/tomiam8/DRC-2019-team1/blob/master/andre_vision.py)

### Program flow (1/2/3 all done in their own threads, with output from previous step):
1. Camera continuously takes pictures (See Camera class) 
    Is a blocking process with lot's of I/O, so theading used so can process images while picture is being taken
2. Process a picture (see HandCodedLaneFollower.follow_lane)
    1. Resize to a very small resolution - Makes it run *much* faster
    2. Convert image to HSV - Makes thresholding for blue/yellow work much better
    3. Threshold to get a seperate blue/yellow lane lines
    4. Use cv2.HoughLinesP to get lines in each image
    5. Split lines for image into three seperate collections, for top / middle / bottom sections of image
    6. For each section, create an 'average' line
         1. Originally, just averaged gradient and intercept
         2. Later, found averaging angle instead of gradient and perpendicular distance from (originally origin, but later used) an x-value of 0, but y-value of middle of section worked better (e.g. for vertical or horizontal lines)
    7. From the equation for the average line (found above), find a point on the line (use y-value of middle of section)
    8. Find the midpoints for the lines
        1. If both the yellow and blue points were found for a section, use the midpoint between them for that section
        2. Otherwise compare the change in x with a midpoint in another section (eg top has blue and yellow points, middle just has a blue point. Use change from top to middle in blue point as change in the midpoint, to get a middle midpoint. Bottom dosen't have any midpoint), or just use a constant offset (if only one line found)
    9. Do PID control-theory stuff to calculate an angle and a speed to drive at
3. Send angle and speed to arduino constantly
    - Another team found using multiprocessing instead of threading for this thread ran faster
