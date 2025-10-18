from __future__ import annotations

from typing import List

from src.models import CarPose, Cone, Path2D


import math


class PathPlanning:
    """Student-implemented path planner.

    You are given the car pose and an array of detected cones, each cone with (x, y, color)
    where color is 0 for yellow (right side) and 1 for blue (left side). The goal is to
    generate a sequence of path points that the car should follow.

    Implement ONLY the generatePath function.
    """

    def __init__(self, car_pose: CarPose, cones: List[Cone], track_width: float = 3.0):
        """Initialize planner.

        track_width: approximate distance between left and right cones (meters).
        """
        self.car_pose = car_pose
        self.cones = cones
        self.track_width = track_width

    def generatePath(self) -> Path2D:
        """Return a list of path points (x, y) in world frame.

        Requirements and notes:
        - Cones: color==0 (yellow) are on the RIGHT of the track; color==1 (blue) are on the LEFT. 
        - You may be given 2, 1, or 0 cones on each side.
        - Use the car pose (x, y, yaw) to seed your path direction if needed.
        - Return a drivable path that stays between left (blue) and right (yellow) cones.
        - The returned path will be visualized by PathTester.

        The path can contain as many points as you like, but it should be between 5-10 meters,
        with a step size <= 0.5. Units are meters.

        This implementation builds a sampled forward path along the car heading up to a
        set distance. For each sample it inspects cones that lie near that longitudinal
        slice and computes a lane center. When only one side is available the missing side
        is inferred using the track width and heading. The path is then smoothed.
        """

        car = self.car_pose
        W = float(self.track_width)

        # Convert cones into lists for quicker access
        left_cones = [c for c in self.cones if c.color == 1]   # blue = left
        right_cones = [c for c in self.cones if c.color == 0]  # yellow = right

        # Path parameters
        total_length = 6.0  # total forward distance (m); between 5-10m per requirements
        step = 0.4  # step size <= 0.5
        num_steps = max(2, int(total_length / step))

        # Precompute heading and perpendicular unit vectors (world frame)
        hx = math.cos(car.yaw)
        hy = math.sin(car.yaw)
        # perp points to the left of heading
        px = -math.sin(car.yaw)
        py = math.cos(car.yaw)

        path: Path2D = []

        # Helper: compute longitudinal (along heading) and lateral (along perp) coords of a cone relative to car
        def cone_coords(c: Cone):
            dx = c.x - car.x
            dy = c.y - car.y
            lon = dx * hx + dy * hy
            lat = dx * px + dy * py
            return lon, lat

        # Build samples along heading
        for i in range(1, num_steps + 1):
            s = i * (total_length / num_steps)
            # default center is straight ahead at distance s
            center_lon = s
            center_lat = 0.0

            # find cones in a longitudinal window around s
            window = step * 1.5
            left_lats = []
            right_lats = []
            for c in left_cones:
                lon, lat = cone_coords(c)
                if lon >= 0.0 and abs(lon - s) <= window:
                    left_lats.append(lat)
            for c in right_cones:
                lon, lat = cone_coords(c)
                if lon >= 0.0 and abs(lon - s) <= window:
                    right_lats.append(lat)

            # Compute center lateral position based on available cones
            if left_lats and right_lats:
                # Use average lateral positions of cones in the slice
                left_avg = sum(left_lats) / len(left_lats)
                right_avg = sum(right_lats) / len(right_lats)
                center_lat = (left_avg + right_avg) / 2.0
            elif left_lats:
                left_avg = sum(left_lats) / len(left_lats)
                # infer right cone by shifting to the right by track width
                center_lat = left_avg - (W / 2.0)
            elif right_lats:
                right_avg = sum(right_lats) / len(right_lats)
                # infer left cone by shifting to the left by track width
                center_lat = right_avg + (W / 2.0)
            else:
                # no cones in this slice; fallback to line along heading
                center_lat = 0.0

            # convert back to world coordinates
            wx = car.x + center_lon * hx + center_lat * px
            wy = car.y + center_lon * hy + center_lat * py
            path.append((wx, wy))

        # Prepend current car position as starting point
        path.insert(0, (car.x, car.y))

        # Simple smoothing: moving average over 3 points
        smooth_path: Path2D = []
        N = len(path)
        for i in range(N):
            sum_x = 0.0
            sum_y = 0.0
            count = 0
            for j in range(max(0, i - 1), min(N, i + 2)):
                sum_x += path[j][0]
                sum_y += path[j][1]
                count += 1
            smooth_path.append((sum_x / count, sum_y / count))

        return smooth_path
 