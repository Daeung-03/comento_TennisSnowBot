"""
경로 계획 및 모션 제어 모듈
"""
def semantic_to_binary_map(semantic_list, rows, cols):
    """
    semantic_list: [[row, col, state], ...]
    return: matrix[row][col] = 1 (can move) or 0 (obstacle)
    """
    matrix = [[0 for _ in range(cols)] for _ in range(rows)]

    for row, col, state in semantic_list:
        # 통행구역
        if state in [0, 2, 3]:     # 자유공간, 제설완료, 흰눈
            matrix[row][col] = 1
        else:                      # 코트라인, 로봇, 로봇은 지금 static한 장애물로 삼음.
            matrix[row][col] = 0

    return matrix

def get_neighbors(pos, matrix):#A*
    """
    pos: (row, col)
    return: list of neighbor positions
    """
    row, col = pos
    neighbors = []
    rows = len(matrix)
    cols = len(matrix[0])

    directions = [(-1,0), (1,0), (0,-1), (0,1)]

    for dr, dc in directions:
        nr, nc = row + dr, col + dc
        if 0 <= nr < rows and 0 <= nc < cols:
            if matrix[nr][nc] == 1:
                neighbors.append((nr, nc))

    return neighbors

def heuristic(a, b):
    """
    a, b: (row, col)
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def a_star(matrix, start, goal):
    """
    matrix: binary grid map
    start, goal: (row, col)
    return: path as list of (row, col)
    """

    open_set = [start]
    closed_set = set()

    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        # f최소 인 점
        current = min(open_set, key=lambda x: f_score.get(x, float('inf')))

        # 도착점
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

    # 경로 없음
    return []



def generate_coverage_from_bbox(top_left, bottom_right, matrix):
    """
    bbox 영역 내에서 간단한 행 스캔 방식 coverage path 생성
    """
    r_min, c_min = top_left
    r_max, c_max = bottom_right

    waypoints = []

    for r in range(r_min, r_max + 1):
        row_cells = []
        for c in range(c_min, c_max + 1):
            if matrix[r][c] == 1:
                row_cells.append((r, c))

        if not row_cells:
            continue

        if r % 2 == 0:
            waypoints.extend(row_cells)
        else:
            waypoints.extend(reversed(row_cells))

    return waypoints



def custom_path_planner(grid, semantic_list, start_loc, clusters):
    """
    제설 경로 계획 알고리즘 (cluster bbox 기반)

    Args:
        grid: AutoNavSim2D 그리드 객체
        semantic_list: 맵 시맨틱 리스트
        start_loc: 시작 위치 (row, col)
        clusters: 인지 모듈에서 전달된 snow cluster bbox 리스트
                  예:
                  [
                    {
                      "top_left": (r1, c1),
                      "bottom_right": (r2, c2)
                    },
                    ...
                  ]

    Returns:
        tuple: (path, runtime)
    """
    import time
    start_time = time.time()

    rows = grid.rows
    cols = grid.cols
    matrix = semantic_to_binary_map(semantic_list, rows, cols)

    full_path = [start_loc]
    current_pos = start_loc

    for cluster in clusters:
        top_left = tuple(cluster["top_left"])
        bottom_right = tuple(cluster["bottom_right"])

        # 1. 클러스터 진입 (좌상단을 entry point로 사용)
        entry_path = a_star(matrix, current_pos, top_left)
        if entry_path:
            full_path.extend(entry_path[1:])
            current_pos = top_left

        # 2. bbox 내부 coverage
        coverage_points = generate_coverage_from_bbox(
            top_left, bottom_right, matrix
        )

        for target in coverage_points:
            if target == current_pos:
                continue

            sub_path = a_star(matrix, current_pos, target)
            if not sub_path:
                continue

            full_path.extend(sub_path[1:])
            current_pos = target

    runtime = f"{int((time.time() - start_time) * 1000)}ms"
    return (full_path, runtime)


def custom_motion_planner(grid, path, start, end):
    """
    제설 모션 제어 계획
    
    Args:
        grid: AutoNavSim2D 그리드
        path: 계획된 경로
        start: 시작 지점
        end: 종료 지점
    
    Returns:
        tuple: (robot_pose, waypoints)
            - robot_pose: 로봇 현재 포즈 (PoseStamped)
            - waypoints: 웨이포인트 리스트
    """
    robot_pose = start
    if path and path[0] == start:
        waypoints = path[1:]
    else:
        waypoints = path[:]
    
    return (robot_pose, waypoints)
