"""Lane-based sensor simulation for the Robot Arcade Engine.

Created by Elena Villalba for Intelligent Robotics, UPC-EEBE.
"""

from dataclasses import dataclass

from . import constants as cfg


@dataclass
class ScanData:
    """Simplified message inspired by sensor_msgs/LaserScan.

    ranges always contains exactly three distances in this order:
    [left adjacent lane, current lane, right adjacent lane].
    """

    ranges: list[float]


@dataclass
class RobotPose:
    x: float = 0.0
    y: float = 0.0
    lane: int = 1


@dataclass
class ObstacleData:
    """One visual obstacle occupying one or two adjacent lanes."""

    lanes: tuple[int, ...]
    y: float
    passed: bool = False

    @property
    def x(self):
        occupied_centres = [cfg.LANE_CENTRES[index] for index in self.lanes]
        return sum(occupied_centres) / len(occupied_centres)

    @property
    def width(self):
        if len(self.lanes) == 1:
            return cfg.OBSTACLE_WIDTH
        left = min(cfg.LANE_CENTRES[index] for index in self.lanes)
        right = max(cfg.LANE_CENTRES[index] for index in self.lanes)
        return right - left + cfg.OBSTACLE_WIDTH

    @property
    def depth(self):
        return cfg.OBSTACLE_DEPTH


def _distance_in_lane(lane, robot_y, obstacles):
    distance = cfg.MAX_SENSOR_DISTANCE
    for obstacle in obstacles:
        if obstacle.passed or lane not in obstacle.lanes:
            continue
        forward_distance = obstacle.y - obstacle.depth / 2 - robot_y
        if forward_distance >= 0:
            distance = min(distance, forward_distance)
    return min(distance, cfg.MAX_SENSOR_DISTANCE)


def create_scan(robot_lane, robot_y, obstacles):
    """Create [left, front, right] relative to the robot's current lane.

    When the robot is on an outer lane, the exterior sensor reports the nearby
    wall. It never sees an obstacle in the far opposite lane.
    """
    left_lane = robot_lane - 1
    right_lane = robot_lane + 1

    if left_lane < 0:
        left_distance = cfg.WALL_DISTANCE
    else:
        left_distance = _distance_in_lane(left_lane, robot_y, obstacles)

    front_distance = _distance_in_lane(robot_lane, robot_y, obstacles)

    if right_lane > 2:
        right_distance = cfg.WALL_DISTANCE
    else:
        right_distance = _distance_in_lane(right_lane, robot_y, obstacles)

    return ScanData(
        ranges=[
            round(left_distance, 3),
            round(front_distance, 3),
            round(right_distance, 3),
        ]
    )

