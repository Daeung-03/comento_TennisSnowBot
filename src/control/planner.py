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

    directions = [(-1,0), (1,0), (0,-1), (0,1)]  # 上下左右

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

def a_star(matrix, start, goal):
    """
    matrix: binary grid map
    start, goal: (row, col)
    return: path as list of (row, col)
    """

    open_set = [start]
    closed_set = set()

    came_from = {}          # child -> parent
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

            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            f_score[neighbor] = tentative_g + heuristic(neighbor, goal)

    # 경로 없음
    return []

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path



def generate_coverage_waypoints_from_semantic(semantic_list):
    """
     state == 3 (흰눈)에 coverage planning path 생성
    """
    snow_cells = [(r, c) for r, c, s in semantic_list if s == 3]

    # 행으로 배열
    snow_cells.sort(key=lambda x: (x[0], x[1]))

    waypoints = []
    current_row = None
    row_buffer = []

    for cell in snow_cells:
        if current_row is None:
            current_row = cell[0]

        if cell[0] != current_row:
            if current_row % 2 == 0:
                waypoints.extend(row_buffer)
            else:
                waypoints.extend(reversed(row_buffer))

            row_buffer = []
            current_row = cell[0]

        row_buffer.append(cell)

    # 마지막 줄 처리
    if row_buffer:
        if current_row % 2 == 0:
            waypoints.extend(row_buffer)
        else:
            waypoints.extend(reversed(row_buffer))

    return waypoints



def custom_path_planner(grid, semantic_list, start_loc, goal_loc):
    """
    제설 경로 계획 알고리즘
    
    Args:
        grid: AutoNavSim2D 그리드 객체
        semantic_list: 맵 매트릭스 (2D list)
        start_loc: 시작 위치
        goal_loc: 목표 위치
    
    Returns:
        tuple: (path, runtime)
            - path: 경로 리스트
            - runtime: 계산 시간 문자열 (예: '10ms')
    """
    import time
    start_time = time.time()
    # binary map 생성
    rows = grid.rows
    cols = grid.cols
    matrix = semantic_to_binary_map(semantic_list, rows, cols)

    # 눈만 처리하는 coverage path 생성
    coverage_points = generate_coverage_waypoints_from_semantic(semantic_list)

    full_path = [start_loc]
    current_pos = start_loc

    for target in coverage_points:
        if target == current_pos:
            continue


        if target in get_neighbors(current_pos, matrix):
            full_path.append(target)
            current_pos = target
        else:
            # A* 로 보충
            sub_path = a_star(matrix, current_pos, target)
            if not sub_path:
                continue

            for p in sub_path[1:]:
                full_path.append(p)

            current_pos = target

    runtime = f"{int((time.time() - start_time)*1000)}ms"
    return full_path, runtime


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
    #robot_pose = None
    #waypoints = []
    robot_pose = start
    waypoints = path
    
    return (robot_pose, waypoints)
