"""
제어 팀원 작업 공간
경로 계획 및 모션 제어 모듈
"""


def custom_path_planner(grid, matrix, start_loc, goal_loc):
    """
    제설 경로 계획 알고리즘
    
    Args:
        grid: AutoNavSim2D 그리드 객체
        matrix: 맵 매트릭스 (2D list, 1=자유공간, 0=장애물/눈)
        start_loc: 시작 위치
        goal_loc: 목표 위치
    
    Returns:
        tuple: (path, runtime)
            - path: 경로 리스트
            - runtime: 계산 시간 문자열 (예: '10ms')
    
    TODO: 제어 팀원이 구현 (A*, Dijkstra 등)
    """
    # 기본값: 빈 경로 반환
    path = []
    runtime = '0ms'
    
    print("WARNING: 기본 경로 계획 함수 사용 중 - 제어 팀원이 구현 필요")
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
    
    TODO: 제어 팀원이 구현
    """
    robot_pose = None
    waypoints = []
    
    print("WARNING: 기본 모션 플래너 사용 중 - 제어 팀원이 구현 필요")
    return (robot_pose, waypoints)
