## Github
- Do this in command line (and push) after adding spotless checking to the github actions `git update-index --chmod=+x ./gradlew`.

## Camera Calibration
Extrinsics - Camera world-relative 3d pose. <br>
Intrinsics - Unique pin-hole camera properties (idk too much about this but it's things like focal length and optical center).

### How a point in world-coordinates is converted to a 2d image-coordinate
The point in world-coordinates is converted to camera-coordinates (camera is the origin) using the camera world-coordinates, also known as the extrinsics. Then, the point in camera-coordinates is converted to a 2d image-coordinate by applying the intrinsics on the point in camera-coordinates.

### Re-projection
Re-projection is done by the same exact process above. A point is converted from world-coordinates to pixel image-coordinates given the extrinsics and the intrinsics of the camera. Re-projection error is the distance in pixels between some re-projected world-coordinate point and that actual point in the frame in image-coordinates.

### SolvePNP
Given multiple points in world-coordinates, the camera's intrinsics, and the points in image-coordinates, solvePnP will find the camera's extrinsics. It will do this by guessing initial extrinsics, and then reprojecting all the world-coordinate points into image-coordinates. SolvePnP works iteratively to reduce the re-projection error between each image point and its corresponding re-projection.

### Camera Calibration
To actually find intrinsics, camera calibration is necessary. A checkerboard pattern is used that is a set of corners with known world-coordinates (the top-left corner is the origin of the world-coordinate system). The calibrator will detect the corners in the image and map the image-coordinates of each corner to its world-coordinates. Many images at different positions of the pattern are taken, and an array of all the combined world-coordinate points and an array of all the combined image-coordinate points is made. Calibration works by guessing intrinsics initially, and then running solvePnP, to get a re-projection error that would be caused by incorrect intrinsics. The calibrator then works iteratively refining the intrinsics and calling solvePnP until the re-projection error is small enough, and at that point the intrinsics are found. The reprojection error for all the combined points can be used to determine how accurate the intrinsics really are.

### Noise
Noise is innaccurate measurements that occur from time to time from a sensor or signal. When detecting corners, there may be some noise in the camera sensor, and the detected image-coordinates of a corner can be different than what they are expected to be given the camera's current extrinsics and calibrated intrinsics.

### Ambiguity
Often when detecting a single tag, solvePnP may come up with two candidate pose solutions for that single tag frame. Each solution has its own reprojection error. Ambiguity occurs when both solutions have very similar reprojection error, meaning both poses fit the image corners equally well, and it can be unclear which of the two candidate poses is the "correct" one. When ambiguity is low, the two reprojection errors are very different, and the two poses differ strongly both in translation and heading, so comparing both poses to the gyro heading (which is always accurate) can find the correct pose (lower reprojection error may not necessarily be the correct pose, it just means the projected tag corners are closer to the image tag corners). In the case where ambiguity is high, the two poses are very similar, and their heading distance from the gyro may be very similar, so after gyro disambiguation, the wrong pose may still be chosen (incorrect translation), therefore it's good to ignore higher ambiguity measurements. Higher noise can also increase ambiguity as the lower reprojection error estimate would increase in reprojection error and be closer to the higher reprojection error estimate.
