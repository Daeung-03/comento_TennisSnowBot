import pickle
import numpy as np
import random
import threading
import time
import math
import pygame
import pkg_resources
from collections import deque
from sklearn.cluster import DBSCAN

# 라이브러리 임포트
from autonavsim2d.autonavsim2d import AutoNavSim2D
from autonavsim2d.utils.utils import BLACK, GREEN, BLUE, GREY, ORANGE, RED, WHITE, generate_waypoints_v4
from autonavsim2d.utils.robot_model import Robot

# 필수 색상 정의
RED, GREEN, BLUE, BLACK, WHITE, ORANGE = (255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 0, 0), (255, 255, 255), (255, 165,
                                                                                                             0)


class DummyLogger:
    def log(self, x): pass

    def get_logs(self): return []


class FakeRect:
    def __init__(self, x, y, w, h):
        self.x, self.y = x, y
        self.width, self.height = w, h


class CustomUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if name == '__rect_constructor': return FakeRect
        return super().find_class(module, name)


def a_star(matrix, start, goal):
    if start == goal: return [start]

    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(pos, matrix):
        r, c = pos
        res = []
        rows, cols = len(matrix), len(matrix[0])
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and matrix[nr][nc] == 1:
                res.append((nr, nc))
        return res

    open_set = [start]
    came_from, g_score = {}, {start: 0}
    f_score = {start: heuristic(start, goal)}
    while open_set:
        current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]
        open_set.remove(current)
        for n in get_neighbors(current, matrix):
            tg = g_score[current] + 1
            if tg < g_score.get(n, float('inf')):
                came_from[n] = current
                g_score[n] = tg
                f_score[n] = tg + heuristic(n, goal)
                if n not in open_set: open_set.append(n)
    return []


class ForcedPixelFollowSim(AutoNavSim2D):
    def __init__(self, filename):
        config = {"show_frame": True, "show_grid": False, "map": filename}
        super().__init__(window='amr', config=config)
        self.path_deque = deque()
        self.full_path = []
        self.clusters_data = {}  # {목표좌표: [소속된 모든 픽셀 좌표들]}
        self.auto_navigate = True
        self.last_spawn_time = time.time()
        self.logger = DummyLogger()
        self.robot = None
        self.robot_angle = 0.0

    def spawn_snow(self, count=1):
        grid = self.map_val
        for _ in range(count):
            rs, cs = random.randint(30, 140), random.randint(30, 190)
            for _ in range(25):
                nr, nc = max(0, min(173, rs + random.randint(-6, 6))), max(0, min(228, cs + random.randint(-6, 6)))
                grid[nr][nc][1] = BLACK

    def update_global_plan(self):
        grid = self.map_val
        rows, cols = len(grid), len(grid[0])
        for r in range(rows):
            for c in range(cols):
                if grid[r][c][1] == ORANGE: grid[r][c][1] = WHITE

        snow_points = [[r, c] for r in range(rows) for c in range(cols) if
                       grid[r][c][1] not in [WHITE, (255, 255, 255), ORANGE]]

        self.path_deque.clear()
        self.full_path = []
        self.clusters_data = {}  # 군집 데이터 초기화

        if not snow_points: return

        dbscan = DBSCAN(eps=10, min_samples=5).fit(snow_points)
        raw_clusters = []

        for label in set(dbscan.labels_):
            if label == -1: continue
            pts = np.array(snow_points)[np.where(dbscan.labels_ == label)[0]]
            # 군집의 목표점 설정 (가장 아래쪽, 가장 왼쪽 지점)
            target = (int(pts[:, 0].max()), int(pts[:, 1].min()))
            raw_clusters.append(target)

            # ⭐ [군집 데이터 저장] 목표점과 해당 군집의 모든 픽셀 연결
            self.clusters_data[target] = [tuple(p) for p in pts]

            color = [RED, GREEN, BLUE, (0, 200, 200)][label % 4]
            for pr, pc in pts: grid[pr][pc][1] = color

        if self.robot and raw_clusters:
            curr_r = max(0, min(rows - 1, int(self.robot.y // 5)))
            curr_c = max(0, min(cols - 1, int(self.robot.x // 5)))
            remaining = raw_clusters[:]
            ordered = []
            temp_p = (curr_r, curr_c)

            while remaining:
                remaining.sort(key=lambda x: (temp_p[0] - x[0]) ** 2 + (temp_p[1] - x[1]) ** 2)
                next_t = remaining.pop(0)
                ordered.append(next_t)
                temp_p = next_t

            matrix = self.generate_grid_matrix(grid)
            last_p = (curr_r, curr_c)
            temp_full_path = []
            for target in ordered:
                segment = a_star(matrix, last_p, target)
                if segment:
                    for p in segment[1:]:
                        # 각 경로 점마다 해당 점이 군집의 최종 목적지인지 표시하기 위해 튜플에 정보 추가
                        is_target = (p == target)
                        self.path_deque.append((p, is_target))
                        temp_full_path.append(p)
                        grid[p[0]][p[1]][1] = ORANGE
                    last_p = target
            self.full_path = temp_full_path

    def run(self):
        run, FPS, clock = True, 60, pygame.time.Clock()
        lasttime = pygame.time.get_ticks()
        m2p = 3779.52
        grid = self.map_val

        start_node = grid[160][20]
        self.robot = Robot((start_node[0].centerx, start_node[0].centery),
                           pkg_resources.resource_filename('autonavsim2d', 'utils/assets/robot_circle_1.png'),
                           0.02117 * m2p)

        self.spawn_snow(count=6)
        self.update_global_plan()

        while run:
            clock.tick(FPS)
            dt = (pygame.time.get_ticks() - lasttime) / 1000
            lasttime = pygame.time.get_ticks()

            for event in pygame.event.get():
                if event.type == pygame.QUIT: run = False

            if time.time() - self.last_spawn_time > 3:
                self.spawn_snow(count=1)
                self.update_global_plan()
                self.last_spawn_time = time.time()

            if self.auto_navigate and self.path_deque:
                (target_r, target_c), is_cluster_end = self.path_deque[0]
                target_x = target_c * 5 + 2.5
                target_y = target_r * 5 + 2.5

                dx, dy = target_x - self.robot.x, target_y - self.robot.y
                self.robot_angle = math.atan2(dy, dx)

                step_size = 2.0
                if math.hypot(dx, dy) > step_size:
                    self.robot.x += math.cos(self.robot_angle) * step_size
                    self.robot.y += math.sin(self.robot_angle) * step_size
                else:
                    self.robot.x, self.robot.y = target_x, target_y

                if hasattr(self.robot, 'rect'):
                    self.robot.rect.center = (self.robot.x, self.robot.y)

                # 도착 판정
                if math.hypot(target_x - self.robot.x, target_y - self.robot.y) < 3:
                    grid[target_r][target_c][1] = WHITE

                    # ⭐ [핵심: 군집 제거 로직]
                    # 만약 현재 도착한 픽셀이 군집의 최종 목적지라면, 해당 군집 전체 삭제
                    if is_cluster_end:
                        target_coord = (target_r, target_c)
                        if target_coord in self.clusters_data:
                            for pr, pc in self.clusters_data[target_coord]:
                                grid[pr][pc][1] = WHITE  # 군집 전체 하얗게 제거
                            print(f"🧹 군집 {target_coord} 제거 완료!")

                    self.path_deque.popleft()
                    if self.full_path: self.full_path.pop(0)

                if not self.path_deque:
                    self.update_global_plan()

            self.draw_path_planning_window(self.ACTIVE_WINDOW, grid, pygame.Rect(0, 0, 0, 0), GREEN,
                                           pygame.Rect(0, 0, 0, 0), BLUE, 0, pygame.Rect(0, 0, 0, 0), RED, self.robot,
                                           dt, True, [self.robot_angle], 0, len(self.path_deque), self.logger, [])

            if self.full_path:
                self.draw_path(self.full_path, grid, None, None, ORANGE, False)

        pygame.quit()


if __name__ == "__main__":
    sim = ForcedPixelFollowSim('blank_map.pkl')
    sim.run()