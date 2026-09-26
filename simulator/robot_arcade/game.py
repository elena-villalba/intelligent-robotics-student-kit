"""Robot Run graphical engine and lane-based arcade simulation.

Created by Elena Villalba for Intelligent Robotics, UPC-EEBE.
Provided for educational use within this course.
"""

from __future__ import annotations

import math
from pathlib import Path
import random

try:
    import pygame
except ImportError:
    pygame = None

from . import constants as cfg
from .sensors import ObstacleData, RobotPose, ScanData, create_scan


def generate_course(seed):
    """Generate 20 reproducible stages with increasing difficulty."""
    rng = random.Random(seed)
    obstacles = []
    world_y = 8.0

    # Stages 1-10: one-lane obstacles. The first one blocks the starting lane.
    single_lane_pattern = (1, 2, 1, 0, 1, 2, 1, 0, 1, 2)
    for lane in single_lane_pattern:
        obstacles.append(ObstacleData(lanes=(lane,), y=world_y))
        world_y += rng.uniform(5.3, 5.8)

    # Stages 11-15: a single wide barrier occupies two adjacent lanes.
    double_barrier_pattern = ((0, 1), (1, 2), (0, 1), (1, 2), (0, 1))
    for lanes in double_barrier_pattern:
        obstacles.append(ObstacleData(lanes=lanes, y=world_y))
        world_y += 5.25

    # Stages 16-20: alternate double barriers and single-lane obstacles.
    final_pattern = ((0, 1), (2,), (1, 2), (0,), (0, 1))
    for lanes in final_pattern:
        obstacles.append(ObstacleData(lanes=lanes, y=world_y))
        world_y += 4.85

    finish_y = world_y + 3.5
    return obstacles, finish_y


class MissionModel:
    """Game state independent from the graphical renderer."""

    def __init__(self, controller, seed):
        self.controller = controller
        self.seed = seed
        self.reset()

    def reset(self):
        self.pose = RobotPose(x=cfg.LANE_CENTRES[1], y=0.0, lane=1)
        self.obstacles, self.finish_y = generate_course(self.seed)
        self.action = cfg.STOP
        self.state = "READY"
        self.score = 0
        self.controller_error = ""
        self.failure_reason = ""
        self.finish_bonus_added = False
        self.decision_time_left = 0.0
        self.countdown_time = 0.0
        self.lane_change_active = False
        self.lane_change_elapsed = 0.0
        self.lane_change_start_x = self.pose.x
        self.target_lane = self.pose.lane

    @property
    def obstacles_passed(self):
        return sum(item.passed for item in self.obstacles)

    @property
    def speed(self):
        passed = self.obstacles_passed
        if passed >= 15:
            return cfg.FINAL_WORLD_SPEED
        if passed >= 10:
            return cfg.HIGH_WORLD_SPEED
        if passed >= 5:
            return cfg.MID_WORLD_SPEED
        return cfg.BASE_WORLD_SPEED

    def start_countdown(self):
        if self.state == "READY":
            self.state = "COUNTDOWN"
            self.countdown_time = 3.5

    def countdown_label(self):
        if self.countdown_time <= 0.5:
            return "GO!"
        return str(max(1, math.ceil(self.countdown_time - 0.5)))

    def scan(self):
        return create_scan(self.pose.lane, self.pose.y, self.obstacles)

    def _request_action(self):
        try:
            action = self.controller(self.scan())
            self.controller_error = ""
        except Exception as exc:  # Student mistakes must not close the engine.
            action = cfg.STOP
            self.controller_error = f"{type(exc).__name__}: {exc}"

        self.action = action if action in cfg.VALID_ACTIONS else cfg.STOP
        self.decision_time_left = cfg.DECISION_INTERVAL

        if self.action == cfg.TURN_LEFT:
            if self.pose.lane == 0:
                self.state = "FAILED"
                self.failure_reason = "WALL COLLISION"
            else:
                self._begin_lane_change(self.pose.lane - 1)
        elif self.action == cfg.TURN_RIGHT:
            if self.pose.lane == 2:
                self.state = "FAILED"
                self.failure_reason = "WALL COLLISION"
            else:
                self._begin_lane_change(self.pose.lane + 1)

    def _begin_lane_change(self, target_lane):
        self.lane_change_active = True
        self.lane_change_elapsed = 0.0
        self.lane_change_start_x = self.pose.x
        self.target_lane = target_lane

    def _update_lane_change(self, dt):
        if not self.lane_change_active:
            return
        self.lane_change_elapsed += dt
        progress = min(1.0, self.lane_change_elapsed / cfg.LANE_CHANGE_DURATION)
        smooth_progress = progress * progress * (3.0 - 2.0 * progress)
        target_x = cfg.LANE_CENTRES[self.target_lane]
        self.pose.x = (
            self.lane_change_start_x
            + (target_x - self.lane_change_start_x) * smooth_progress
        )
        if progress >= 1.0:
            self.pose.x = target_x
            self.pose.lane = self.target_lane
            self.lane_change_active = False
            self.decision_time_left = 0.0

    def _collides_with_obstacle(self):
        for obstacle in self.obstacles:
            if obstacle.passed:
                continue
            obstacle_left = obstacle.x - obstacle.width / 2
            obstacle_right = obstacle.x + obstacle.width / 2
            obstacle_near = obstacle.y - obstacle.depth / 2
            obstacle_far = obstacle.y + obstacle.depth / 2
            horizontal_collision = (
                self.pose.x + cfg.ROBOT_RADIUS >= obstacle_left
                and self.pose.x - cfg.ROBOT_RADIUS <= obstacle_right
            )
            vertical_collision = (
                self.pose.y + cfg.ROBOT_RADIUS >= obstacle_near
                and self.pose.y - cfg.ROBOT_RADIUS <= obstacle_far
            )
            if horizontal_collision and vertical_collision:
                return True
        return False

    def _update_score(self):
        for obstacle in self.obstacles:
            if not obstacle.passed and self.pose.y > obstacle.y + 0.7:
                obstacle.passed = True
                self.score += cfg.POINTS_PER_OBSTACLE

    def step(self, dt):
        if self.state == "COUNTDOWN":
            self.countdown_time -= dt
            if self.countdown_time <= 0.0:
                self.state = "RUNNING"
                self.action = cfg.FORWARD
                self.decision_time_left = 0.0
            return

        if self.state != "RUNNING":
            return

        if not self.lane_change_active:
            self.decision_time_left -= dt
            if self.decision_time_left <= 0.0:
                self._request_action()
                if self.state == "FAILED":
                    return

        self._update_lane_change(dt)

        if self.action != cfg.STOP:
            self.pose.y += self.speed * dt

        if self._collides_with_obstacle():
            self.state = "FAILED"
            self.failure_reason = "OBSTACLE COLLISION"
            self.action = cfg.STOP
            return

        self._update_score()
        if self.pose.y >= self.finish_y:
            self.state = "WON"
            self.action = cfg.STOP
            if not self.finish_bonus_added:
                self.score += cfg.FINISH_BONUS
                self.finish_bonus_added = True


class RobotGame:
    """Public game class imported by the students."""

    def __init__(
        self,
        robot_name,
        controller,
        sensor_reporter=None,
        team_number=0,
        seed=None,
    ):
        self.robot_name = robot_name
        self.controller = controller
        self.sensor_reporter = sensor_reporter
        self.team_number = int(team_number)
        self.seed = int(seed if seed is not None else 2026 + self.team_number)
        self.mode = "TEST"
        self.test_number = 0
        self.debug = False
        self.running = False
        self.test_pose = RobotPose(lane=1)
        self.test_obstacles = []
        self.test_scan = ScanData([cfg.MAX_SENSOR_DISTANCE] * 3)
        self.test_action = cfg.STOP
        self.controller_error = ""
        self.mission = MissionModel(controller, self.seed)
        self.camera_y = 0.0

    def _ensure_pygame(self):
        if pygame is None:
            raise RuntimeError(
                "Pygame is not installed. Run: sudo apt install python3-pygame"
            )

    def _safe_controller(self, scan):
        try:
            action = self.controller(scan)
            self.controller_error = ""
        except Exception as exc:
            action = cfg.STOP
            self.controller_error = f"{type(exc).__name__}: {exc}"
        return action if action in cfg.VALID_ACTIONS else cfg.STOP

    def _create_test_scenario(self):
        rng = random.Random(self.seed + self.test_number * 97)
        robot_lane = self.test_number % 3
        self.test_pose = RobotPose(
            x=cfg.LANE_CENTRES[robot_lane],
            y=0.0,
            lane=robot_lane,
        )

        obstacle_patterns = (
            (),
            (0,),
            (1,),
            (2,),
            (0, 1),
            (1, 2),
        )
        visible_lanes = list(
            obstacle_patterns[self.test_number % len(obstacle_patterns)]
        )

        distance = rng.uniform(1.2, 4.6)
        self.test_obstacles = []
        if visible_lanes:
            visible_lanes.sort()
            # Adjacent occupied lanes are drawn as a single wide obstacle.
            if len(visible_lanes) >= 2 and all(
                visible_lanes[index + 1] - visible_lanes[index] == 1
                for index in range(len(visible_lanes) - 1)
            ):
                self.test_obstacles.append(
                    ObstacleData(lanes=tuple(visible_lanes), y=distance)
                )
            else:
                for lane in visible_lanes:
                    self.test_obstacles.append(
                        ObstacleData(lanes=(lane,), y=distance)
                    )

        self.test_scan = create_scan(
            self.test_pose.lane,
            self.test_pose.y,
            self.test_obstacles,
        )
        self.test_action = self._safe_controller(self.test_scan)

        if self.sensor_reporter is not None:
            try:
                self.sensor_reporter(self.test_scan)
            except Exception as exc:
                self.controller_error = f"{type(exc).__name__}: {exc}"

        self.test_number += 1

    def run(self):
        self._ensure_pygame()
        pygame.init()
        pygame.display.set_caption("Robot Run · Intelligent Robotics")
        self.screen = pygame.display.set_mode(
            (cfg.SCREEN_WIDTH, cfg.SCREEN_HEIGHT)
        )
        self.clock = pygame.time.Clock()
        self.font_small = pygame.font.SysFont("monospace", 18, bold=True)
        self.font = pygame.font.SysFont("monospace", 25, bold=True)
        self.font_big = pygame.font.SysFont("monospace", 46, bold=True)
        self._load_assets()
        self.running = True
        self._create_test_scenario()

        while self.running:
            dt = min(self.clock.tick(cfg.FPS) / 1000.0, 0.05)
            self._handle_events()
            if self.mode == "GAME":
                self.mission.step(dt)
                self._update_camera(dt)
            self._draw()
            pygame.display.flip()

        pygame.quit()

    def _load_assets(self):
        logo_path = Path(__file__).parent / "assets" / "upc_logo.png"
        self.upc_logo = None
        if logo_path.exists():
            original = pygame.image.load(str(logo_path)).convert_alpha()
            self.upc_logo = pygame.transform.scale(original, (145, 44))

    def _reset_mission(self, start=False):
        self.mission.reset()
        self.camera_y = 0.0
        if start:
            self.mission.start_countdown()

    def _update_camera(self, dt):
        """Follow the robot only after it has visibly travelled forwards."""
        target_y = max(0.0, self.mission.pose.y - cfg.CAMERA_FOLLOW_DISTANCE)
        follow_amount = min(1.0, dt * cfg.CAMERA_FOLLOW_SPEED)
        self.camera_y += (target_y - self.camera_y) * follow_amount

    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_t:
                    self.mode = "TEST"
                    self._create_test_scenario()
                elif event.key == pygame.K_g:
                    self.mode = "GAME"
                    self._reset_mission()
                elif event.key == pygame.K_d:
                    self.debug = not self.debug
                elif event.key == pygame.K_r:
                    if self.mode == "GAME":
                        self._reset_mission()
                    else:
                        self.test_number = 0
                        self._create_test_scenario()
                elif event.key == pygame.K_SPACE:
                    if self.mode == "TEST":
                        self._create_test_scenario()
                    elif self.mission.state == "READY":
                        self.mission.start_countdown()
                    elif self.mission.state in {"FAILED", "WON"}:
                        self._reset_mission(start=True)

    def _text(self, text, x, y, color=cfg.INK, font=None, centre=False):
        image = (font or self.font).render(str(text), True, color)
        rect = image.get_rect()
        if centre:
            rect.center = (round(x), round(y))
        else:
            rect.topleft = (round(x), round(y))
        self.screen.blit(image, rect)
        return rect

    def _project(self, world_x, world_y, progress):
        forward = world_y - progress
        if forward <= 0.05 or forward > cfg.VIEW_DISTANCE:
            return None
        scale = 1.0 / (1.0 + 0.24 * forward)
        screen_x = cfg.PLAY_CENTRE_X + world_x * 170.0 * scale
        screen_y = cfg.HORIZON_Y + (
            cfg.ROBOT_SCREEN_Y - cfg.HORIZON_Y
        ) * scale
        return screen_x, screen_y, scale, forward

    def _draw_dotted_line(self, start, end, color, dot=7, gap=8, width=2):
        delta_x = end[0] - start[0]
        delta_y = end[1] - start[1]
        length = math.hypot(delta_x, delta_y)
        if length < 1:
            return
        unit_x = delta_x / length
        unit_y = delta_y / length
        position = 0.0
        while position < length:
            dot_end = min(position + dot, length)
            pygame.draw.line(
                self.screen,
                color,
                (start[0] + unit_x * position, start[1] + unit_y * position),
                (start[0] + unit_x * dot_end, start[1] + unit_y * dot_end),
                width,
            )
            position += dot + gap

    def _draw_dashed_rect(self, rect, color=cfg.INK, width=3):
        self._draw_dotted_line(rect.topleft, rect.topright, color, 7, 6, width)
        self._draw_dotted_line(rect.topright, rect.bottomright, color, 7, 6, width)
        self._draw_dotted_line(rect.bottomright, rect.bottomleft, color, 7, 6, width)
        self._draw_dotted_line(rect.bottomleft, rect.topleft, color, 7, 6, width)

    def _sensor_color(self, distance):
        if distance <= cfg.DANGER_DISTANCE:
            return cfg.RED
        if distance <= cfg.WARNING_DISTANCE:
            return cfg.YELLOW
        return cfg.GREEN

    def _draw_header(self, mode, score=0, obstacles=0, goal=0):
        pygame.draw.line(self.screen, cfg.INK, (18, 73), (1262, 73), 5)
        self._text("ROBOT RUN", 25, 14, font=self.font_big)
        if mode == "TEST":
            self._text("SENSOR & DECISION TEST", 350, 28, cfg.MID_GREY)
        else:
            self._text(f"SCORE {score:03d}", 330, 28)
            self._text(f"OBSTACLES {obstacles}/20", 520, 28)
            self._text(f"GOAL {goal:02d}%", 785, 28)

    def _draw_road(self, progress):
        # Outer road edges.
        for road_x in (-cfg.CORRIDOR_HALF_WIDTH, cfg.CORRIDOR_HALF_WIDTH):
            points = []
            for step in range(1, 73):
                point = self._project(road_x, progress + step * 0.5, progress)
                if point:
                    points.append((point[0], point[1]))
            if len(points) > 1:
                pygame.draw.lines(self.screen, cfg.INK, False, points, 5)

        # Dashed lines separate the three lanes.
        dash_period = 2.25
        first_dash = math.floor(progress / dash_period) * dash_period
        for boundary_x in cfg.LANE_BOUNDARIES:
            segment_start = first_dash
            while segment_start < progress + cfg.VIEW_DISTANCE:
                start = self._project(
                    boundary_x,
                    segment_start,
                    progress,
                )
                end = self._project(
                    boundary_x,
                    segment_start + 1.1,
                    progress,
                )
                if start and end:
                    width = max(2, int(5 * start[2]))
                    pygame.draw.line(
                        self.screen,
                        cfg.MID_GREY,
                        (start[0], start[1]),
                        (end[0], end[1]),
                        width,
                    )
                segment_start += dash_period

        # World-anchored roadside posts move past the robot as it advances.
        post_period = 4.0
        first_post = math.floor(progress / post_period) * post_period
        post_y = first_post
        while post_y < progress + cfg.VIEW_DISTANCE:
            for road_x in (-cfg.CORRIDOR_HALF_WIDTH, cfg.CORRIDOR_HALF_WIDTH):
                point = self._project(road_x, post_y, progress)
                if point:
                    height = max(4, int(42 * point[2]))
                    width = max(2, int(9 * point[2]))
                    pygame.draw.rect(
                        self.screen,
                        cfg.INK,
                        (point[0] - width / 2, point[1] - height, width, height),
                    )
            post_y += post_period

    def _draw_obstacle(self, obstacle, progress):
        projection = self._project(obstacle.x, obstacle.y, progress)
        if projection is None:
            return
        x, y, scale, _ = projection
        width = max(10, int(obstacle.width * 145 * scale))
        height = max(10, int(125 * scale))
        rect = pygame.Rect(0, 0, width, height)
        rect.midbottom = (round(x), round(y))
        pygame.draw.rect(self.screen, cfg.BACKGROUND, rect)
        pygame.draw.rect(self.screen, cfg.INK, rect, max(3, int(7 * scale)))

        # Pixel hazard pattern.
        tile = max(4, int(14 * scale))
        for tile_x in range(rect.left + tile, rect.right - tile, tile):
            color = cfg.INK if (tile_x // tile) % 2 == 0 else cfg.LIGHT_GREY
            pygame.draw.rect(
                self.screen,
                color,
                (tile_x, rect.top + tile, tile, max(4, rect.height - 2 * tile)),
            )

    def _draw_finish(self, finish_y, progress):
        left = self._project(-2.8, finish_y, progress)
        right = self._project(2.8, finish_y, progress)
        if not left or not right:
            return
        scale = min(left[2], right[2])
        post_height = max(16, int(175 * scale))
        top_y = min(left[1], right[1]) - post_height
        pygame.draw.line(self.screen, cfg.INK, (left[0], left[1]), (left[0], top_y), 6)
        pygame.draw.line(self.screen, cfg.INK, (right[0], right[1]), (right[0], top_y), 6)
        banner = pygame.Rect(left[0], top_y - 8, right[0] - left[0], max(12, int(42 * scale)))
        pygame.draw.rect(self.screen, cfg.BACKGROUND, banner)
        pygame.draw.rect(self.screen, cfg.INK, banner, 4)
        square = max(4, int(12 * scale))
        columns = max(1, banner.width // square)
        for column in range(columns):
            for row in range(2):
                color = cfg.INK if (column + row) % 2 == 0 else cfg.WHITE
                pygame.draw.rect(
                    self.screen,
                    color,
                    (banner.left + column * square, banner.top + row * square, square, square),
                )
        if banner.width > 85:
            label = pygame.Rect(banner.centerx - 42, banner.centery - 12, 84, 24)
            pygame.draw.rect(self.screen, cfg.BACKGROUND, label)
            self._text("FINISH", label.centerx, label.centery, font=self.font_small, centre=True)

    def _robot_screen_x(self, pose):
        return cfg.PLAY_CENTRE_X + pose.x * 155

    def _robot_screen_position(self, pose, camera_y=None):
        if camera_y is None:
            return self._robot_screen_x(pose), cfg.ROBOT_SCREEN_Y
        projection = self._project(pose.x, pose.y, camera_y)
        if projection is None:
            return self._robot_screen_x(pose), cfg.ROBOT_SCREEN_Y
        return projection[0], projection[1]

    def _draw_robot(self, pose, action, camera_y=None):
        surface = pygame.Surface((126, 112), pygame.SRCALPHA)
        body = pygame.Rect(30, 28, 66, 66)
        pygame.draw.rect(surface, cfg.INK, (13, 37, 18, 54))
        pygame.draw.rect(surface, cfg.INK, (95, 37, 18, 54))
        pygame.draw.rect(surface, cfg.BACKGROUND, body)
        pygame.draw.rect(surface, cfg.INK, body, 6)
        pygame.draw.rect(surface, cfg.INK, (50, 45, 26, 11))
        pygame.draw.line(surface, cfg.INK, (63, 28), (63, 13), 5)
        pygame.draw.rect(surface, cfg.INK, (57, 7, 13, 9))
        for x in (51, 62, 73):
            pygame.draw.rect(surface, cfg.INK, (x, 76, 6, 6))
        robot_x, robot_y = self._robot_screen_position(pose, camera_y)
        rect = surface.get_rect(center=(robot_x, robot_y))
        self.screen.blit(surface, rect)

    def _draw_sensor_paths(self, pose, scan):
        start = (self._robot_screen_x(pose), cfg.ROBOT_SCREEN_Y - 27)
        lane_targets = (pose.lane - 1, pose.lane, pose.lane + 1)
        for distance, lane in zip(scan.ranges, lane_targets):
            if lane < 0:
                target_x = -cfg.CORRIDOR_HALF_WIDTH
            elif lane > 2:
                target_x = cfg.CORRIDOR_HALF_WIDTH
            else:
                target_x = cfg.LANE_CENTRES[lane]
            target_y = pose.y + max(0.45, distance)
            projection = self._project(target_x, target_y, pose.y)
            if projection:
                self._draw_dotted_line(
                    start,
                    (projection[0], projection[1]),
                    self._sensor_color(distance),
                    6,
                    8,
                    2,
                )

    def _draw_panel(self, scan, action, error=""):
        pygame.draw.line(self.screen, cfg.INK, (936, 94), (936, 655), 5)
        panel = pygame.Rect(956, 106, 302, 252)
        self._draw_dashed_rect(panel, width=3)
        self._text("SENSORS (m)", panel.centerx, 136, font=self.font, centre=True)

        labels = ("LEFT", "FRONT", "RIGHT")
        columns = (995, 1106, 1213)
        for label, distance, x in zip(labels, scan.ranges, columns):
            self._text(label, x, 194, cfg.MID_GREY, self.font_small, True)
            pygame.draw.circle(self.screen, self._sensor_color(distance), (x, 226), 7)
            self._text(f"{distance:.2f}", x, 270, font=self.font, centre=True)

        decision = pygame.Rect(956, 392, 302, 108)
        self._draw_dashed_rect(decision, width=3)
        self._text("DECISION", decision.centerx, 422, cfg.MID_GREY, self.font_small, True)
        self._text(action, decision.centerx, 467, font=self.font, centre=True)

        if error:
            self._text("CONTROLLER ERROR", 956, 536, cfg.RED, self.font_small)
            words = error.split()
            line = ""
            y = 562
            for word in words:
                candidate = f"{line} {word}".strip()
                if len(candidate) > 26:
                    self._text(line, 956, y, cfg.RED, self.font_small)
                    y += 22
                    line = word
                else:
                    line = candidate
            self._text(line, 956, y, cfg.RED, self.font_small)

    def _draw_overlay(self, title, subtitle, color=cfg.INK):
        rect = pygame.Rect(180, 282, 570, 150)
        pygame.draw.rect(self.screen, cfg.BACKGROUND, rect)
        pygame.draw.rect(self.screen, cfg.INK, rect, 6)
        self._text(title, rect.centerx, rect.top + 49, color, self.font_big, True)
        self._text(subtitle, rect.centerx, rect.top + 108, font=self.font, centre=True)

    def _draw_collision_effect(self, pose, camera_y=None):
        centre_x, centre_y = self._robot_screen_position(pose, camera_y)
        centre_y -= 18
        pygame.draw.line(self.screen, cfg.RED, (centre_x - 48, centre_y - 48), (centre_x + 48, centre_y + 48), 8)
        pygame.draw.line(self.screen, cfg.RED, (centre_x + 48, centre_y - 48), (centre_x - 48, centre_y + 48), 8)

    def _draw_debug_map(self, pose, obstacles):
        rect = pygame.Rect(24, 95, 194, 224)
        pygame.draw.rect(self.screen, cfg.BACKGROUND, rect)
        pygame.draw.rect(self.screen, cfg.INK, rect, 4)
        self._text("DEBUG", rect.centerx, 116, cfg.MID_GREY, self.font_small, True)
        road_left = rect.centerx - 66
        road_right = rect.centerx + 66
        pygame.draw.line(self.screen, cfg.INK, (road_left, 145), (road_left, 306), 3)
        pygame.draw.line(self.screen, cfg.INK, (road_right, 145), (road_right, 306), 3)
        for boundary in (-22, 22):
            self._draw_dotted_line((rect.centerx + boundary, 145), (rect.centerx + boundary, 306), cfg.MID_GREY, 5, 5, 2)
        for obstacle in obstacles:
            relative_y = obstacle.y - pose.y
            if 0 <= relative_y <= 15:
                x = rect.centerx + obstacle.x * 20
                y = 289 - relative_y * 9
                width = max(12, obstacle.width * 20)
                pygame.draw.rect(self.screen, cfg.INK, (x - width / 2, y - 5, width, 10))
        robot_x = rect.centerx + pose.x * 20
        pygame.draw.rect(self.screen, cfg.RED, (robot_x - 6, 283, 12, 12))

    def _draw_footer(self):
        self._text(
            "T: TEST   G: GAME   SPACE: NEXT/START   R: RESET   D: DEBUG",
            18,
            681,
            cfg.MID_GREY,
            self.font_small,
        )
        tiny_font = pygame.font.SysFont("monospace", 13, bold=True)
        self._text(
            "Created by Elena Villalba",
            1105,
            644,
            cfg.MID_GREY,
            tiny_font,
            centre=True,
        )

        if self.upc_logo:
            logo_rect = self.upc_logo.get_rect(center=(1105, 592))
            self.screen.blit(self.upc_logo, logo_rect)

    def _draw(self):
        self.screen.fill(cfg.BACKGROUND)
        if self.mode == "TEST":
            self._draw_test()
        else:
            self._draw_game()
        self._draw_footer()

    def _draw_test(self):
        self._draw_header("TEST")
        self._draw_road(self.test_pose.y)
        for obstacle in self.test_obstacles:
            self._draw_obstacle(obstacle, self.test_pose.y)
        self._draw_sensor_paths(self.test_pose, self.test_scan)
        self._draw_robot(self.test_pose, self.test_action)
        self._draw_panel(self.test_scan, self.test_action, self.controller_error)
        self._text(
            f"TEST {self.test_number:02d} · TEAM {self.team_number:02d} · "
            f"ROBOT {self.robot_name.upper()}",
            28,
            91,
            cfg.MID_GREY,
            self.font_small,
        )
        if self.debug:
            self._draw_debug_map(self.test_pose, self.test_obstacles)

    def _draw_game(self):
        mission = self.mission
        goal = int(min(100, max(0, 100 * mission.pose.y / mission.finish_y)))
        self._draw_header(
            "GAME",
            score=mission.score,
            obstacles=mission.obstacles_passed,
            goal=goal,
        )
        self._text(
            f"TEAM {self.team_number:02d} · ROBOT {self.robot_name.upper()}",
            28,
            91,
            cfg.MID_GREY,
            self.font_small,
        )
        self._draw_road(self.camera_y)
        self._draw_finish(mission.finish_y, self.camera_y)
        for obstacle in mission.obstacles:
            if not obstacle.passed:
                self._draw_obstacle(obstacle, self.camera_y)
        scan = mission.scan()
        self._draw_robot(mission.pose, mission.action, self.camera_y)
        self._draw_panel(scan, mission.action, mission.controller_error)

        if mission.state == "READY":
            self._draw_overlay("READY?", "Press SPACE to start")
        elif mission.state == "COUNTDOWN":
            self._draw_overlay(mission.countdown_label(), "Get ready!")
        elif mission.state == "FAILED":
            self._draw_collision_effect(mission.pose, self.camera_y)
            self._draw_overlay(
                "MISSION FAILED",
                f"Score {mission.score} · Press SPACE",
                cfg.RED,
            )
        elif mission.state == "WON":
            self._draw_overlay(
                "MISSION COMPLETE!",
                f"Final score {mission.score} · Press SPACE",
                cfg.GREEN,
            )
        if self.debug:
            self._draw_debug_map(mission.pose, mission.obstacles)
