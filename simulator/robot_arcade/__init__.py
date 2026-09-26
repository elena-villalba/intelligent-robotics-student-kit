"""Public interface of the Robot Arcade Engine.

Created by Elena Villalba for Intelligent Robotics, UPC-EEBE.
"""

from .constants import (
    AUTHOR,
    COURSE,
    FORWARD,
    STOP,
    TURN_LEFT,
    TURN_RIGHT,
    VERSION,
)
from .game import RobotGame

__all__ = [
    "RobotGame",
    "FORWARD",
    "TURN_LEFT",
    "TURN_RIGHT",
    "STOP",
]
__author__ = AUTHOR
__version__ = VERSION

