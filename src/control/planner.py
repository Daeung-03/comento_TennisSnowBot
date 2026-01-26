"""
경로 계획 및 모션 제어 모듈
"""
<<<<<<< HEAD
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
    return (full_path, runtime)
=======
def update_matrix_for_snow(matrix, snow_list, except_point):
    """
    Update binary grid map by marking snow regions as traversable.

    이 함수는 기존 matrix에서 장애물(0)로 표시된 snow 영역과
    except_point에 해당하는 좌표들을 통행 가능 영역(1)로 수정한다.

    Parameters
    ----------
    matrix : list[list[int]]
        Binary grid map (1: free space, 0: obstacle)

    snow_list : list
        List of snow clusters.
        Each cluster is defined by [(top_left), (bottom_right)] coordinates.

    except_point : list[tuple]
        List of snow points (row, col) that should be ignored but remain traversable.

    Returns
    -------
    list[list[int]]
        Updated binary grid map where snow areas are walkable.
    """
    new_matrix = [row[:] for row in matrix]

    # snow cluster 내 모두 통행 가능으로 표시
    for (top_left, bottom_right) in snow_list:
        r1, c1 = top_left
        r2, c2 = bottom_right
        for r in range(r1, r2 + 1):
            for c in range(c1, c2 + 1):
                new_matrix[r][c] = 1

    # except point도 통행가능으로 표시
    for r, c in except_point:
        new_matrix[r][c] = 1

    return new_matrix


def get_neighbors(pos, matrix):#A*
    """
    Get valid neighbor cells for A* search.

    A* 알고리즘에서 현재 위치(pos) 기준으로
    상/하/좌/우 방향의 이동 가능한 이웃 셀을 반환한다.

    Parameters
    ----------
    pos : tuple
        Current position (row, col)

    matrix : list[list[int]]
        Binary grid map

    Returns
    -------
    list[tuple]
        List of neighboring positions that are traversable.
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
    Heuristic function for A* (Manhattan distance).

    A* 알고리즘에서 사용하는 휴리스틱 함수로,
    두 좌표 사이의 Manhattan distance를 계산한다.

    Parameters
    ----------
    a, b : tuple
        Grid coordinates (row, col)

    Returns
    -------
    int
        Estimated cost from a to b.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def reconstruct_path(came_from, current):
    """
    Reconstruct path from A* search result.

    A* 탐색 종료 후, came_from dictionary를 이용하여
    start 지점부터 goal 지점까지의 경로를 복원한다.

    Parameters
    ----------
    came_from : dict
        Dictionary storing parent nodes

    current : tuple
        Goal position

    Returns
    -------
    list[tuple]
        Reconstructed path from start to goal.
    """
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def a_star(matrix, start, goal):
    """
    A* path planning on a grid map.

    Grid-based 환경에서 start에서 goal까지
    최단 경로를 계산하기 위한 A* 알고리즘 구현이다.

    Parameters
    ----------
    matrix : list[list[int]]
        Binary grid map (1: free, 0: obstacle)

    start : tuple
        Start position (row, col)

    goal : tuple
        Goal position (row, col)

    Returns
    -------
    list[tuple]
        Path as a list of grid coordinates.
        If no path exists, returns empty list.
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

def find_nearest_cluster(matrix, start, snow_list):
    """
    Find the nearest snow cluster using A* distance.

    각 snow cluster의 좌상단과 우하단을
    entry point로 설정하고, A*를 이용해 가장 가까운 cluster를 선택한다.

    Parameters
    ----------
    matrix : list[list[int]]
        Updated binary grid map

    start : tuple
        Current robot position (row, col)

    snow_list : list
        List of remaining snow clusters

    Returns
    -------
    tuple
        (nearest_cluster, path_to_cluster)
    """
    best_path = None
    best_cluster = None
    min_len = float('inf')

    for cluster in snow_list:
        entry_points = [
            cluster[0],  # 좌상단
            cluster[1],  # 우하단
        ]

        for ep in entry_points:
            path = a_star(matrix, start, ep)
            if path and len(path) < min_len:
                min_len = len(path)
                best_path = path
                best_cluster = cluster

    return best_cluster, best_path


def generate_cluster_coverage_path(cluster, entry_point):
    """
    Generate coverage path inside a snow cluster starting from entry point.

    Parameters
    ----------
    cluster : list[tuple]
        [(top_left), (bottom_right)] coordinates of a snow cluster

    entry_point : tuple
        Entry point into the cluster (must be one of the corners)

    Returns
    -------
    list[tuple]
        Coverage path starting from entry_point and covering the entire cluster.
    """
    (r1, c1), (r2, c2) = cluster

    # 기본 row-scan (좌상단 → 우하단)
    path = []
    for r in range(r1, r2 + 1):
        if (r - r1) % 2 == 0:
            for c in range(c1, c2 + 1):
                path.append((r, c))
        else:
            for c in range(c2, c1 - 1, -1):
                path.append((r, c))

    # entry point가 좌상단이면 그대로 사용
    if entry_point == (r1, c1):
        return path

    # entry point가 우하단이면 반대로 사용
    if entry_point == (r2, c2):
        return path[::-1]

    # 그 외의 경우 (안전 장치)
    # entry가 예상 위치가 아닐 경우 기본 path 반환
    return path
>>>>>>> feature/control





def custom_path_planner(start_point, matrix, snow_list, except_point):
    """
    Custom global path planner for snow removal.

    이 함수는 snow removal task를 위한 전체 경로를 생성한다.
    전체 동작은 다음과 같은 단계로 구성된다:

    1) Snow 영역을 통행 가능하도록 matrix를 수정
    2) 현재 위치에서 가장 가까운 snow cluster를 A*로 탐색
    3) Cluster 내부를 coverage path planning으로 완전 커버
    4) 모든 snow cluster가 처리될 때까지 반복

    Parameters
    ----------
    start_point : tuple
        Robot start position (row, col)

    matrix : list[list[int]]
        Original binary grid map

    snow_list : list
        List of snow clusters to be covered

    except_point : list[tuple]
        Snow points that should be ignored during planning

    Returns
    -------
    list[tuple]
        Final global path including inter-cluster paths
        and intra-cluster coverage paths.
    """
<<<<<<< HEAD
    robot_pose = start
    if path and path[0] == start:
        waypoints = path[1:]
    else:
        waypoints = path[:]
    
    return (robot_pose, waypoints)
=======
    final_path = []
    current_pos = start_point

    # 1. 맵 수정
    matrix = update_matrix_for_snow(matrix, snow_list, except_point)

    remaining_clusters = snow_list[:]

    while remaining_clusters:
        # 2. 최근거리 클러스터 찾기
        cluster, path_to_cluster = find_nearest_cluster(
            matrix, current_pos, remaining_clusters
        )

        if cluster is None or path_to_cluster is None:
            break

        # 3. 클러스터 입구
        final_path.extend(path_to_cluster)
        current_pos = path_to_cluster[-1]

        # 4. 클러스터 coverage
        entry_point = path_to_cluster[-1]

        coverage_path = generate_cluster_coverage_path(cluster,entry_point)

        final_path.extend(coverage_path[1:])
        current_pos = coverage_path[-1]

        # 5. 처리된 클러스터 제거
        remaining_clusters.remove(cluster)

    return final_path
>>>>>>> feature/control
