"""
경로 계획 및 모션 제어 모듈
"""
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

    각 snow cluster의 네 개의 코너를
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
        (nearest_cluster, path_to_cluster, entry_point)
    """
    best_path = None
    best_cluster = None
    best_entry = None
    min_len = float('inf')

    for cluster in snow_list:
        (r1, c1), (r2, c2) = cluster
        entry_points = [
            (r1, c1), #좌상단
            (r1, c2), #우상단
            (r2, c1), #좌하단
            (r2, c2) #우하단
        ]

        for ep in entry_points:
            path = a_star(matrix, start, ep)
            if path and len(path) < min_len:
                min_len = len(path)
                best_path = path
                best_cluster = cluster
                best_entry = ep
    return best_cluster, best_path, best_entry


def generate_cluster_coverage_path(cluster, entry_point):
    """
    Parameters
    ----------
    cluster : list[tuple]
        [(top_left), (bottom_right)] coordinates of a snow cluster

    entry_point : tuple
        Entry point into the cluster (boundary point)

    Returns
    -------
    list[tuple]
        Coverage path that starts naturally from the entry side and
        covers the entire cluster.
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
    final_path = []
    current_pos = start_point

    # 1. 맵 수정
    matrix = update_matrix_for_snow(matrix, snow_list, except_point)

    remaining_clusters = snow_list[:]

    while remaining_clusters:
        # 2. 최근거리 클러스터 찾기
        cluster, path_to_cluster, entry_point = find_nearest_cluster(
            matrix, current_pos, remaining_clusters
        )

        if cluster is None or path_to_cluster is None:
            break

        # 3. 클러스터 입구
        if final_path and final_path[-1] == path_to_cluster[0]:
            final_path.extend(path_to_cluster[1:])
        else:
            final_path.extend(path_to_cluster)

        current_pos = path_to_cluster[-1]

        # 4. 클러스터 coverage
        coverage_path = generate_cluster_coverage_path(cluster, entry_point)

        if final_path and final_path[-1] == coverage_path[0]:
            final_path.extend(coverage_path[1:])
        else:
            final_path.extend(coverage_path)

        current_pos = coverage_path[-1]

        # 5. 처리된 클러스터 제거
        remaining_clusters.remove(cluster)

    return final_path



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
