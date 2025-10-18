
 Overview
This project implements a center line path planner for a car navigating between blue and yellow cones (left and right track boundaries).
The algorithm estimates a smooth, drivable path by averaging cone positions and generating waypoints in front of the car.

1. Coordinate Transformation
   Each cone’s position is expressed relative to the car using its heading (yaw).
   This helps the planner reason in the car’s forward direction.

2. Longitudinal Sampling
   The algorithm builds equally spaced path points along the car’s heading direction.
   For each step, it looks at cones that fall within a small window around that distance.

3. Lateral Position Estimation

   If both blue and yellow cones exist in the current window, the centerline is placed halfway between them.
   If only one side exists, the path is shifted by half the track width toward the missing side.
   If no cones are visible, the path continues straight ahead.

4. World Coordinate Reconstruction
   Each local (longitudinal, lateral) waypoint is converted back into global (x, y) coordinates for visualization.

 Known Limitations

Sparse cones: When cones are far apart or missing, the path may drift or go slightly outside the lane.
Sharp turns:The straight-line sampling may not bend fast enough, causing corner clipping.
Window sensitivity:The path depends heavily on the distance window used to select nearby cones.
No memory: When no cones are visible, the car continues straight instead of maintaining the previous direction.



 Possible Improvements

Curve fitting(e.g. polynomial or spline interpolation).
Smoothing filters to make transitions less jerky.
Memory of last lateral offset to handle missing cones gracefully.

