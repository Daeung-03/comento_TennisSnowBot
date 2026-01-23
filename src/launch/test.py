"""
군집 자동 순회 통합 테스트

[현재 상태]
- 인지 모듈: 완료 ✅ (군집화로 목적지 생성)
- 제어 모듈: 부분 완료 🔄 (A* 경로 계획만 작동)
  → motion planner는 제어 팀원 작업 필요

[제어 팀원 TODO]
- custom_motion_planner 구현
- 또는 custom_planner에서 반환하는 경로 형식 수정
"""

import sys
import os

project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, project_root)

import numpy as np
from sklearn.cluster import DBSCAN

from src.perception.detect import get_snow_area_list, get_perception_info
from src.control.planner import a_star
from autonavsim2d.autonavsim2d import AutoNavSim2D
from autonavsim2d.utils.utils import BLACK, GREEN, BLUE, GREY, ORANGE, RED


# ============================================================
# 설정
# ============================================================

MAP_FILE = 'maps/snow_removal_area_multi3.pkl'
EPS = 10                # 군집화 반경
MIN_SAMPLES = 5         # 최소 샘플 수


# ============================================================
# 유틸리티 함수
# ============================================================

def find_nearest_free_space(matrix, target_r, target_c, max_distance=20):
    """
    목표 좌표 근처의 이동 가능한 셀 찾기
    군집 중심이 장애물일 경우 접근 가능한 인접 좌표 반환
    
    Args:
        matrix: binary grid (1=이동가능, 0=장애물)
        target_r, target_c: 목표 좌표
        max_distance: 최대 탐색 거리
    
    Returns:
        (r, c): 이동 가능한 좌표
    """
    rows = len(matrix)
    cols = len(matrix[0])
    
    # 목표가 이미 이동 가능하면 그대로 반환
    if 0 <= target_r < rows and 0 <= target_c < cols:
        if matrix[target_r][target_c] == 1:
            return (target_r, target_c)
    
    # 나선형 탐색
    for distance in range(1, max_distance + 1):
        for dr in range(-distance, distance + 1):
            for dc in range(-distance, distance + 1):
                if abs(dr) + abs(dc) != distance:
                    continue
                
                nr = target_r + dr
                nc = target_c + dc
                
                if 0 <= nr < rows and 0 <= nc < cols:
                    if matrix[nr][nc] == 1:
                        return (nr, nc)
    
    return (target_r, target_c)


# ============================================================
# 전역 변수 (커스텀 플래너에서 사용)
# ============================================================

cluster_waypoints = []  # 군집 목적지 리스트
binary_matrix = None    # 맵의 binary matrix


# ============================================================
# 커스텀 경로 계획 함수
# ============================================================

def multi_cluster_path_planner(grid, matrix, start_loc, goal_loc):
    """
    여러 군집을 순회하는 경로 계획
    팀원의 A* 알고리즘 사용
    
    [현재 상태]
    - 경로 계획: 작동 ✅
    - 로봇 이동: 작동 안 함 ❌
    
    [제어 팀원 TODO]
    - 반환 형식 수정 또는
    - custom_motion_planner 구현
    
    Args:
        grid: AutoNavSim2D 그리드 객체
        matrix: 맵 매트릭스
        start_loc: 시작 위치
        goal_loc: 목표 위치 (현재는 사용 안 함)
    
    Returns:
        (path, time_taken): 경로 리스트와 계산 시간(초)
    """
    import time
    start_time = time.time()
    
    global cluster_waypoints, binary_matrix
    
    if not cluster_waypoints:
        return ([], 0.0)
    
    # 시작 위치 추출
    if hasattr(start_loc, 'pose'):
        start_r = start_loc.pose.position.r
        start_c = start_loc.pose.position.c
    elif isinstance(start_loc, (list, tuple)) and len(start_loc) >= 2:
        start_r, start_c = int(start_loc[0]), int(start_loc[1])
    else:
        return ([], 0.0)
    
    current_pos = (start_r, start_c)
    
    print(f"\n{'='*70}")
    print(f"🗺️  군집 순회 경로 계획 (팀원의 A* 알고리즘)")
    print(f"{'='*70}")
    print(f"시작: {current_pos}")
    print(f"목표: {len(cluster_waypoints)}개 군집 순회\n")
    
    full_path = [[start_r, start_c]]
    
    # 모든 군집을 순서대로 순회
    for i, target in enumerate(cluster_waypoints):
        print(f"[{i+1}/{len(cluster_waypoints)}] {current_pos} → {target}", end=" ")
        
        # 팀원의 A* 알고리즘 호출
        segment_path = a_star(binary_matrix, current_pos, target)
        
        if not segment_path:
            print("❌ 경로 없음")
            continue
        
        # 경로 추가 (중복 제거)
        for point in segment_path:
            point_list = [int(point[0]), int(point[1])]
            if not full_path or full_path[-1] != point_list:
                full_path.append(point_list)
        
        current_pos = target
        print(f"✅ {len(segment_path)}칸")
    
    time_taken = time.time() - start_time
    
    print(f"\n{'='*70}")
    print(f"✅ 경로 계획 완료!")
    print(f"   전체 경로: {len(full_path)}개 웨이포인트")
    print(f"   계산 시간: {time_taken*1000:.1f}ms")
    print(f"{'='*70}\n")
    
    # 경로 시각화 (노란색)
    for point in full_path:
        try:
            grid.map_val[point[0]][point[1]][1] = ORANGE
        except:
            pass
    
    return (full_path, time_taken)


# TODO: 제어 팀원 작업 필요
def custom_motion_planner_placeholder(grid, path, start, end):
    """
    커스텀 모션 제어 함수 (미완성)
    
    [제어 팀원 TODO]
    이 함수를 구현하여 AutoNavSim2D에 전달
    
    Args:
        grid: AutoNavSim2D 그리드
        path: 계획된 경로
        start: 시작 포즈
        end: 종료 포즈
    
    Returns:
        (robot_pose, waypoints): 로봇 포즈와 웨이포인트
    """
    # 현재는 사용하지 않음 (기본 motion planner 사용)
    pass


# ============================================================
# 메인 실행
# ============================================================

if __name__ == "__main__":
    print("=" * 70)
    print(" " * 10 + "군집 자동 순회 통합 테스트")
    print("=" * 70)
    print("\n[현재 상태]")
    print("  ✅ 인지: 군집화 완료")
    print("  🔄 제어: 경로 계획만 작동 (motion은 제어 팀원 작업 필요)")
    print("=" * 70)
    
    # ========================================
    # 1단계: 인지 - 군집화
    # ========================================
    print("\n[1단계] 인지 모듈 - 눈 감지 및 군집화")
    print("-" * 70)
    
    raw_points_list = get_snow_area_list(MAP_FILE)
    
    if not raw_points_list:
        print(f"❌ {MAP_FILE}에서 데이터를 찾을 수 없습니다.")
        exit(1)
    
    print(f"  ✅ 눈 좌표: {len(raw_points_list)}개")
    
    raw_points = np.array(raw_points_list)
    dbscan = DBSCAN(eps=EPS, min_samples=MIN_SAMPLES).fit(raw_points)
    result = get_perception_info(raw_points_list, dbscan.labels_, raw_points)
    
    cluster_centers = [tuple(c['target_center']) for c in result['clusters']]
    
    print(f"  ✅ 군집 감지: {len(cluster_centers)}개")
    
    if len(cluster_centers) == 0:
        print("  ❌ 군집이 없습니다.")
        exit(1)
    
    # ========================================
    # 2단계: 제어 - 시뮬레이터 준비
    # ========================================
    print("\n[2단계] 제어 모듈 - 시뮬레이터 준비")
    print("-" * 70)
    
    config = {"show_frame": True, "show_grid": False, "map": MAP_FILE}
    
    nav = AutoNavSim2D(
        custom_planner=multi_cluster_path_planner,  # 커스텀 경로 계획
        custom_motion_planner='default',            # 기본 motion planner 사용 (임시)
        window='amr',
        config=config
    )
    
    binary_matrix = nav.generate_grid_matrix(nav.map_val)
    
    # 군집 중심을 접근 가능한 좌표로 변환
    print(f"  🎯 군집 접근 가능 좌표 변환:")
    cluster_waypoints = []
    
    for i, center in enumerate(cluster_centers):
        accessible = find_nearest_free_space(binary_matrix, center[0], center[1])
        cluster_waypoints.append(accessible)
        
        if center == accessible:
            print(f"     {i+1}. {accessible} (접근 가능)")
        else:
            distance = abs(center[0] - accessible[0]) + abs(center[1] - accessible[1])
            print(f"     {i+1}. {center} → {accessible} (보정: {distance}칸)")
    
    print(f"  ✅ {len(cluster_waypoints)}개 목적지 준비 완료")
    
    # ========================================
    # 3단계: 통합 - 시각화
    # ========================================
    print("\n[3단계] 통합 - 군집 시각화")
    print("-" * 70)
    
    colors = [RED, GREEN, BLUE, ORANGE]
    
    # 원래 군집 중심 (작은 점)
    for i, (r, c) in enumerate(cluster_centers):
        color = colors[i % len(colors)]
        try:
            nav.map_val[r][c][1] = color
        except IndexError:
            pass
    
    # 접근 가능한 목적지 (큰 마커)
    for i, (r, c) in enumerate(cluster_waypoints):
        color = colors[i % len(colors)]
        for dr in range(-2, 3):
            for dc in range(-2, 3):
                try:
                    nav.map_val[r + dr][c + dc][1] = color
                except IndexError:
                    pass
    
    print(f"  ✅ {len(cluster_waypoints)}개 군집 마커 표시 완료")
    
    # ========================================
    # 4단계: 실행
    # ========================================
    print("\n" + "=" * 70)
    print("  🚀 시뮬레이터 시작")
    print("=" * 70)
    print("")
    print("  📋 사용법:")
    print("     1️⃣  좌클릭 → 로봇 시작 위치 설정")
    print("     2️⃣  좌클릭 → 아무 곳이나 클릭 (트리거)")
    print("     3️⃣  터미널에서 경로 계획 결과 확인")
    print("  🎯 군집 순서:")
    for i, target in enumerate(cluster_waypoints):
        color_name = ["빨강", "초록", "파랑", "오렌지"][i % 4]
        print(f"     {i+1}. {target} ({color_name} 마커)")
    
    print("\n[시뮬레이터 실행 중]")
    print("→ 경로 계획은 작동하지만")
    print("→ 로봇 이동은 제어 팀원 작업 필요\n")
    
    nav.run()
