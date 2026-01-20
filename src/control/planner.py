import time
"""
경로 계획 및 모션 제어 모듈
"""


def custom_path_planner(grid, matrix, start_loc, goal_loc):
    """
    제설 경로 계획 알고리즘
    
    Args:
        grid: AutoNavSim2D 그리드 객체
        matrix: 맵 매트릭스 (2D list, 1=자유공간, 0=장애물/눈) 장애물을 0이 아닌 다른 수로 표시 필요
        start_loc: 시작 위치
        goal_loc: 목표 위치
    
    Returns:
        tuple: (path, runtime)
            - path: 경로 리스트
            - runtime: 계산 시간 문자열 (예: '10ms')
    """

    start_time = time.time()
    # 기본값: 빈 경로 반환
    path = []
    #runtime = '0ms'
    rows = len(matrix)
    cols = len(matrix[0])
    # --------------------------------------------------
    # 1. 제설 대상(눈) 셀 수집
    # --------------------------------------------------
    snow_cells = []
    for r in range(rows):
        for c in range(cols):
            if matrix[r][c] == 0:
                snow_cells.append((r, c))

    # 눈이 없으면 경로 없음
    if not snow_cells:
        return [], "0ms"

    # --------------------------------------------------
    # 2. 제설 영역 bounding box 계산
    # --------------------------------------------------
    snow_rows = [r for (r, c) in snow_cells]
    snow_cols = [c for (r, c) in snow_cells]

    min_row = min(snow_rows)
    max_row = max(snow_rows)
    min_col = min(snow_cols)
    max_col = max(snow_cols)

    # --------------------------------------------------
    # 3. 시작 위치 → 제설 영역 입구
    # (단순화: 바로 이동한다고 가정) A*(or dijkstra)을 이용해서 start -> entry 거리계획 생성
    # --------------------------------------------------
    current = start_loc
    path.append(current)

    entry_point = (min_row, min_col)
    if current != entry_point:
        path.append(entry_point)

    # --------------------------------------------------
    # 4. Lawn-mower 방식 제설 경로 생성
    # --------------------------------------------------
    direction = 1  # 1: left → right, -1: right → left

    for r in range(min_row, max_row + 1):
        if direction == 1:
            col_range = range(min_col, max_col + 1)
        else:
            col_range = range(max_col, min_col - 1, -1)

        for c in col_range:
            if matrix[r][c] == 0:
                path.append((r, c))

        direction *= -1  # 방향 전환

    # --------------------------------------------------
    # 5. 실행 시간 계산
    # --------------------------------------------------
    runtime_ms = int((time.time() - start_time) * 1000)
    runtime = f"{runtime_ms}ms"
    
    return (path, runtime)


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
