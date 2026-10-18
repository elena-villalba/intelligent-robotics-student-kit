"""Constants used by the Robot Arcade Engine.

Created by Elena Villalba for Intelligent Robotics, UPC-EEBE.
Provided for educational use within this course.
"""

AUTHOR = "Elena Villalba"
COURSE = "Intelligent Robotics · UPC-EEBE"
VERSION = "2.3.0"

# High-level commands available to the students.
FORWARD = "FORWARD"
TURN_LEFT = "TURN_LEFT"
TURN_RIGHT = "TURN_RIGHT"
STOP = "STOP"
VALID_ACTIONS = {FORWARD, TURN_LEFT, TURN_RIGHT, STOP}

# Window and layout.
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 720
PLAY_WIDTH = 930
PANEL_LEFT = 950
FPS = 60
HORIZON_Y = 118
ROBOT_SCREEN_Y = 620
PLAY_CENTRE_X = PLAY_WIDTH // 2
CAMERA_FOLLOW_DISTANCE = 1.65
CAMERA_FOLLOW_SPEED = 3.0

# Simplified arcade movement. The robot stays oriented towards the finish and
# moves smoothly between lanes.
ROBOT_RADIUS = 0.34
LANE_CHANGE_DURATION = 0.42
DECISION_INTERVAL = 0.12
BASE_WORLD_SPEED = 2.25
MID_WORLD_SPEED = 2.75
HIGH_WORLD_SPEED = 3.20
FINAL_WORLD_SPEED = 3.55

# World geometry.
CORRIDOR_HALF_WIDTH = 3.25
LANE_CENTRES = (-2.05, 0.0, 2.05)
LANE_BOUNDARIES = (-1.025, 1.025)
OBSTACLE_WIDTH = 1.05
OBSTACLE_DEPTH = 0.85
VIEW_DISTANCE = 36.0
NUMBER_OF_OBSTACLES = 20
POINTS_PER_OBSTACLE = 10
FINISH_BONUS = 50

# Virtual scan. Students receive only [left, front, right]. LEFT and RIGHT
# refer to the immediately adjacent lane, never to the far opposite lane.
MAX_SENSOR_DISTANCE = 8.0
WARNING_DISTANCE = 3.70
DANGER_DISTANCE = 0.72
WALL_DISTANCE = 0.30

# Retro monochrome palette with three small sensor accents.
BACKGROUND = (249, 249, 245)
INK = (29, 29, 29)
MID_GREY = (101, 101, 98)
LIGHT_GREY = (218, 218, 211)
PANEL = (239, 239, 233)
WHITE = (255, 255, 255)
GREEN = (47, 164, 91)
YELLOW = (232, 169, 32)
RED = (213, 57, 55)
