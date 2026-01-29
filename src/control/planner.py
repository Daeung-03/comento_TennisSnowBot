"""
planner.py - 제설 로봇 경로 계획 및 모션 제어
"""

import time
import math

# ==================== Helper Functions ====================

def update_matrix_for_court_and_snow(matrix: list, snow_list: list) -> list:
    """
    제설 작업을 위한 맵 통행 가능 영역(Matrix) 업데이트

    1. 네트를 제외한 다른 장애물(코트 외곽선, 내부 라인, 눈 등)을 통행 가능(1)으로 변경

    Parameters:
        matrix: 원본 그리드 맵 데이터 (0: 장애물, 1: 이동가능) [[x,y, state], [x,y, state], ... ]
        snow_list: 감지된 눈 클러스터 리스트 [((좌상단 x, y),(우하단 x, y)), ((좌상단 x, y),(우하단 x, y)), ... ]

    Returns:
        list: 업데이트된 2D 그리드 맵 [[x,y, state], [x,y, state], ... ]
    """
    new_matrix = [row[:] for row in matrix]
    rows = len(new_matrix)
    cols = len(new_matrix[0])

    court_cells_changed = 0
    
    if snow_list:
        # 1. 모든 클러스터를 포함하는 Bounding Box (코트 영역)
        all_r1 = min(cluster[0][0] for cluster in snow_list)
        all_c1 = min(cluster[0][1] for cluster in snow_list)
        all_r2 = max(cluster[1][0] for cluster in snow_list)
        all_c2 = max(cluster[1][1] for cluster in snow_list)
        
        # 코트 영역 (+ 20px)
        court_r1 = max(0, all_r1 - 20)
        court_c1 = max(0, all_c1 - 20) # 왼쪽 사이드라인 근처
        court_r2 = min(rows - 1, all_r2 + 20)
        court_c2 = min(cols - 1, all_c2 + 20) # 오른쪽 사이드라인 근처
        
        # 2. 네트 위치 추정 (중앙)
        net_row_approx = (court_r1 + court_r2) // 2
        net_thickness = 4
        
        # 3. 우회 경로(Passage) 확보 범위 설정
        # 사이드라인보다 더 바깥쪽으로 20픽셀 정도 추가 공간을 뚫어줌
        passage_margin = 30
        safe_c1 = max(0, court_c1 - passage_margin)
        safe_c2 = min(cols - 1, court_c2 + passage_margin)
        
        # 4. 영역 설정 루프
        # court_r1 ~ court_r2 (세로 전체), safe_c1 ~ safe_c2 (가로 확장 범위)
        for r in range(court_r1, court_r2 + 1):
            for c in range(safe_c1, safe_c2 + 1):
                
                # (A) 네트 위치인지 확인
                is_net_row = abs(r - net_row_approx) <= net_thickness
                
                # (B) 코트 내부(사이드라인 안쪽)인지 확인
                # 실제 코트 너비보다 약간 좁게 잡아서 네트 부분만 정확히 장애물로 남김
                is_inside_court_width = (court_c1 - 5 <= c <= court_c2 + 5)
                
                # 조건: 네트 위치이면서 동시에 코트 안쪽이면 -> 장애물 유지 (건너뜀)
                if is_net_row and is_inside_court_width:
                    continue
                
                # 그 외 모든 영역(일반 바닥, 라인, 네트 옆 통로) -> 통행 가능(1)
                if new_matrix[r][c] == 0:
                    new_matrix[r][c] = 1
                    court_cells_changed += 1

    # 5. 눈 영역 확인 (혹시 네트 위에 눈이 찍혔을 경우를 대비)
    snow_cells_changed = 0
    for (top_left, bottom_right) in snow_list:
        r1, c1 = top_left
        r2, c2 = bottom_right
        for r in range(r1, r2 + 1):
            for c in range(c1, c2 + 1):
                if 0 <= r < rows and 0 <= c < cols:
                    if new_matrix[r][c] == 0:
                        new_matrix[r][c] = 1
                        snow_cells_changed += 1
    
    return new_matrix


def get_neighbors(pos: tuple, matrix: list) -> list:
    """
    A* 알고리즘용 인접 셀 탐색 (상하좌우)
    
    Parameters:
        pos: 현재 좌표 튜플 (row, col)
        matrix: 맵 데이터 (2D List)

    Returns:
        list: 이동 가능한 인접 좌표 리스트
            [(r1, c1), (r2, c2), ...]
    """
    row, col = pos
    neighbors = []
    rows = len(matrix)
    cols = len(matrix[0])
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    for dr, dc in directions:
        nr, nc = row + dr, col + dc
        if 0 <= nr < rows and 0 <= nc < cols:
            if matrix[nr][nc] == 1:
                neighbors.append((nr, nc))
    
    return neighbors


def heuristic(a: tuple, b: tuple) -> int:
    """
    Manhattan 거리 계산 (Heuristic 함수)
    
    Parameters:
        a: 좌표 A (r, c)
        b: 좌표 B (r, c)
    
    Returns:
        int: 맨해튼 거리 (|x1-x2| + |y1-y2|)
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def reconstruct_path(came_from: dict, current: tuple) -> list:
    """
    A* 탐색 완료 후 경로 역추적
    
    Parameters:
        came_from: 경로 추적용 딕셔너리 {child_node: parent_node}
        current: 목표 지점 좌표 (r, c)

    Returns:
        list: 시작점부터 목표점까지의 경로 리스트
              [(r_start, c_start), ..., (r_goal, c_goal)]
    """
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def a_star(matrix: list, start: tuple, goal: tuple) -> list:
    """
    A* 알고리즘을 이용한 최단 경로 탐색
    
    Parameters:
        matrix: 맵 데이터 (2D List)
        start: 시작 좌표 (r, c)
        goal: 목표 좌표 (r, c)

    Returns:
        list: 경로 좌표 리스트 (실패 시 빈 리스트 [])
              [(r1, c1), (r2, c2), ...]
    """
    if not (0 <= start[0] < len(matrix) and 0 <= start[1] < len(matrix[0])):
        return []
    if not (0 <= goal[0] < len(matrix) and 0 <= goal[1] < len(matrix[0])):
        return []
    
    if matrix[start[0]][start[1]] == 0:
        return []
    if matrix[goal[0]][goal[1]] == 0:
        return []
    
    open_set = [start]
    closed_set = set()
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while open_set:
        current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
        
        if current == goal:
            return reconstruct_path(came_from, current)
        
        open_set.remove(current)
        closed_set.add(current)
        
        for neighbor in get_neighbors(current, matrix):
            if neighbor in closed_set:
                continue
            
            tentative_g = g_score[current] + 1
            
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                if neighbor not in open_set:
                    open_set.append(neighbor)
    
    return []


def find_nearest_cluster(matrix: list, start: tuple, snow_list: list) -> tuple:
    """
    현재 위치에서 가장 가까운 눈 클러스터 및 진입점 탐색
    
    Parameters:
        matrix: 맵 데이터 (2D List)
        start: 현재 로봇 위치 (r, c)
        snow_list: 남은 눈 클러스터 리스트
                  [((r_min, c_min), (r_max, c_max)), ...]

    Returns:
        tuple: (최적 클러스터, 이동 경로 리스트, 진입 좌표)
               ( ((r1,c1),(r2,c2)), [(r,c)...], (entry_r, entry_c) )
    """
    best_path = None
    best_cluster = None
    best_entry = None
    min_len = float('inf')
    
    for cluster in snow_list:
        (r1, c1), (r2, c2) = cluster
        # 클러스터의 4개 코너를 진입 후보점으로
        entry_points = [(r1, c1), (r1, c2), (r2, c1), (r2, c2)]
        
        for ep in entry_points:
            path = a_star(matrix, start, ep)
            if path and len(path) < min_len:
                min_len = len(path)
                best_path = path
                best_cluster = cluster
                best_entry = ep
    
    return best_cluster, best_path, best_entry


def generate_cluster_coverage_path(cluster: tuple, entry_point: tuple) -> list:
    """
    클러스터 내부를 완전히 청소하는 Boustrophedon 경로 생성
    
    Parameters:
        cluster: 눈 클러스터 영역 
                 ((r_min, c_min), (r_max, c_max))
        entry_point: 진입한 모서리 좌표 (r, c)

    Returns:
        list: 청소 경로, 좌표의 리스트 형태
              [(r, c), (r, c+1), ...]
    """
    (r1, c1), (r2, c2) = cluster
    er, ec = entry_point
    
    rows = list(range(r1, r2 + 1))
    cols = list(range(c1, c2 + 1))
    
    if er == r2:
        rows = rows[::-1]
    
    path = []
    for i, r in enumerate(rows):
        if ec == c2:
            col_iter = cols[::-1] if i % 2 == 0 else cols[:]
        else:
            col_iter = cols[:] if i % 2 == 0 else cols[::-1]
        
        path.extend((r, c) for c in col_iter)
    
    return path


def calculate_angle(prev: tuple, curr: tuple) -> float:
    """
    연속된 두 그리드 셀의 이동 방향을 각도(Radian)로 변환
    
    Parameters:
        prev: 이전 좌표 (r, c)
        curr: 현재 좌표 (r, c)

    Returns:
        float: 각도 (라디안, 0 ~ 2pi)
    """
    dr = curr[0] - prev[0]
    dc = curr[1] - prev[1]
    
    if dr > 0:
        return math.pi * 3 / 2
    elif dr < 0:
        return math.pi / 2
    elif dc > 0:
        return 0
    elif dc < 0:
        return math.pi
    else:
        return math.pi / 2


# ==================== Factory Function ====================

def create_snow_removal_planners(snow_clusters: list, debug_mode: bool = False) -> tuple:
    """
    경로 생성기 및 모션 제어기 팩토리 함수
    
    Parameters:
        snow_clusters: 감지된 전체 눈 클러스터 정보 리스트
            [((r_min, c_min), (r_max, c_max)), ...]
        debug_mode: True일 경우 경로 생성 과정 로그로 출력

    Returns:
        tuple: (custom_path_planner 함수, custom_motion_planner 함수)
    """
    
    # 전체 경로를 캐싱하기 위한 리스트
    cached_full_path = []
    path_generated = False
    
    def custom_path_planner(grid, matrix: list, start_point: tuple, end_point: tuple) -> tuple:
        """
        Global Path Planner 함수
        
        최초 호출 시 전체 경로를 생성하여 캐싱하고, 
        이후 호출 시 현재 위치 기반 남은 경로만 반환합니다(autonavsim2D 작동 고려)
        
        Parameters:
            grid: 시뮬레이터 Grid 객체
            matrix: 맵 통행 데이터 (2D List)
            start_point: 시작 좌표 (r, c)
            end_point: 목표 좌표 (사용되지 않음, 시뮬레이터 인터페이스 맞춤용)

        Returns:
            tuple: (경로 리스트 [(r,c)...], 소요 시간 float)
        """
        nonlocal cached_full_path, path_generated
        
        start_time = time.time()

        def log(msg: str):
            if debug_mode: print(msg)
        
        # 이미 전체 경로가 생성되었다면 남은 경로 반환
        if path_generated and cached_full_path:
            log(f"\n🔄 재계획 요청 감지 - 캐시된 경로 사용")
            log(f"   - 현재 위치: {start_point}")
            
            # 현재 위치에서 가장 가까운 남은 경로 지점 찾기
            try:
                start_idx = cached_full_path.index(start_point)
                remaining_path = cached_full_path[start_idx:]
                log(f" - 남은 경로: {len(remaining_path)}개")
                runtime = time.time() - start_time
                return remaining_path, runtime
            except ValueError:
                # 현재 위치가 경로에 없으면 전체 경로 반환
                log(f" - 전체 경로 반환: {len(cached_full_path)}개")
                runtime = time.time() - start_time
                return cached_full_path, runtime
        
        # 최초 경로 생성
        log(f"\n{'='*60}")
        log(f"🎯 [Planner] 전체 경로 계획 시작")
        log(f" - 시작 위치: {start_point}")
        log(f" - 제설 클러스터: {len(snow_clusters)}개")
        
        # 시작 위치 검증
        sr, sc = start_point
        if matrix[sr][sc] == 0:
            log(f" ⚠️ 시작 위치가 장애물 -> 대체 위치 탐색 중...")
            for radius in range(1, 20):
                found = False
                for dr in range(-radius, radius+1):
                    for dc in range(-radius, radius+1):
                        nr, nc = sr + dr, sc + dc
                        if 0 <= nr < len(matrix) and 0 <= nc < len(matrix[0]):
                            if matrix[nr][nc] == 1:
                                start_point = (nr, nc)
                                sr, sc = nr, nc
                                found = True
                                break
                    if found:
                        break
                if found:
                    break
        
        # 코트와 눈 영역을 통행 가능하도록 수정
        updated_matrix = update_matrix_for_court_and_snow(matrix, snow_clusters)
        
        # 전체 경로 생성
        final_path = [start_point]
        current_pos = start_point
        remaining_clusters = snow_clusters[:]
        
        cluster_count = 0
        while remaining_clusters:
            cluster, path_to_cluster, entry_point = find_nearest_cluster(
                updated_matrix, current_pos, remaining_clusters
            )
            
            if cluster is None or path_to_cluster is None:
                break
            
            if final_path[-1] == path_to_cluster[0]:
                final_path.extend(path_to_cluster[1:])
            else:
                final_path.extend(path_to_cluster)
            
            current_pos = path_to_cluster[-1]
            
            coverage_path = generate_cluster_coverage_path(cluster, entry_point)
            
            if final_path[-1] == coverage_path[0]:
                final_path.extend(coverage_path[1:])
            else:
                final_path.extend(coverage_path)
            
            # 진행상황
            log(f" - 클러스터 #{cluster_count+1} 처리 완료 (남은 수: {len(remaining_clusters)-1})")
            
            current_pos = coverage_path[-1]
            remaining_clusters.remove(cluster)
            cluster_count += 1
        
        # 전체 경로 캐싱
        cached_full_path = final_path
        path_generated = True
        
        runtime = time.time() - start_time
        log(f"\n🎯 [Planner] 전체 경로 생성 완료")
        log(f" - 총 Waypoint: {len(final_path)}")
        log(f" - 소요 시간: {runtime:.3f}초")
        log(f"{'='*60}\n")
        
        return final_path, runtime
    
    
    def custom_motion_planner(grid, path: list, start_coord: tuple, end_coord: tuple) -> tuple:
        """
        Motion Planner 함수
        
        그리드 경로를 실제 로봇이 주행할 Waypoint(Pose)로 변환합니다.
        
        Parameters:
            grid: 시뮬레이터 Grid 객체
            path: 계산된 경로 리스트 [(r, c), ...]
            start_coord: 시작 좌표 정보 (Rect, (r, c))
            end_coord: 목표 좌표 정보 (Rect, (r, c))

        Returns:
            tuple: (robot_pose, waypoints)
                   - robot_pose: 초기 로봇 위치 (Pose 객체)
                   - waypoints: 이동할 웨이포인트 리스트 [PoseStamped, ...]
        """
        # 내부 클래스 정의(Autonavsim2D)
        class Position:
            def __init__(self, x, y):
                self.x = x
                self.y = y
        
        class Orientation:
            def __init__(self, w):
                self.w = w
        
        class Pose:
            def __init__(self, position, orientation):
                self.position = position
                self.orientation = orientation
        
        class PoseStamped:
            def __init__(self, pose):
                self.pose = pose

        def log(msg: str):
            if debug_mode: print(msg)
        
        # 시작 위치 설정
        start_rect = start_coord[0]
        start_row, start_col = start_coord[2]
        
        robot_pose = Pose(
            position=Position(
                x=start_rect.x + start_rect.width // 2,
                y=start_rect.y + start_rect.height // 2
            ),
            orientation=Orientation(w=math.pi / 2)
        )
        
        log(f"\n🚀 [Motion] Waypoint 생성 요청")
        log(f"   - 시작 위치: ({start_row}, {start_col})")
        log(f" - 입력 경로 길이: {len(path)}")
        
        waypoints = []
        
        for i, (row, col) in enumerate(path):
            if row >= len(grid) or col >= len(grid[0]):
                continue
            
            cell = grid[row][col]
            rect = cell[0]
            
            # 방향 계산
            if i == 0:
                angle = math.pi / 2
            else:
                prev_row, prev_col = path[i - 1]
                angle = calculate_angle((prev_row, prev_col), (row, col))
            
            waypoint = PoseStamped(
                pose=Pose(
                    position=Position(
                        x=rect.x + rect.width // 2,
                        y=rect.y + rect.height // 2
                    ),
                    orientation=Orientation(w=angle)
                )
            )
            waypoints.append(waypoint)
        
        log(f"✅ [Motion] 생성 완료: {len(waypoints)}개 Waypoints\n")
        
        return robot_pose, waypoints
    
    return custom_path_planner, custom_motion_planner