"""
전체 시스템 통합 실행 파일
인지(눈 감지) → 제어(경로 계획) → 시뮬레이션
"""

import sys
import os
import numpy as np
from sklearn.cluster import DBSCAN

# 프로젝트 루트를 경로에 추가
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, project_root)

from src.perception.detect import (
    get_snow_area_list,
    get_perception_info
)
from src.launch.wrapper import TennisCourtSimulator


def run_full_pipeline(
    map_file, 
    eps=10, 
    min_samples=5,
    visualize_clusters=True,  # 군집 시각화 옵션
    show_targets=True
):
    """
    전체 파이프라인 실행
    
    Args:
        map_file: .pkl 맵 파일 경로
        eps: DBSCAN 군집화 반경
        min_samples: DBSCAN 최소 샘플 수
        visualize_clusters: True면 군집 목적지를 맵에 표시
        show_targets: True면 목적지 좌표 출력
    """
    print("=" * 70)
    print(" " * 20 + "전체 파이프라인 시작")
    print("=" * 70)
    
    # ========================================
    # [1단계] 인지: 눈 감지 및 군집화
    # ========================================
    print("\n[1단계] 인지 모듈 실행")
    print("-" * 70)
    
    try:
        raw_points_list = get_snow_area_list(map_file)
    except FileNotFoundError:
        print(f"❌ 파일을 찾을 수 없습니다: {map_file}")
        print("   다음 경로를 확인하세요:")
        print("   - maps/파일명.pkl")
        print("   - custom_maps/파일명.pkl")
        return None
    
    if not raw_points_list:
        print(f"❌ {map_file}에서 눈 좌표를 찾을 수 없습니다.")
        print("   맵에 검은색 장애물이 없을 수 있습니다.")
        return None
    
    print(f"  ✅ 눈 좌표 로드: {len(raw_points_list)}개")
    
    # 군집화 실행
    raw_points = np.array(raw_points_list)
    dbscan = DBSCAN(eps=eps, min_samples=min_samples).fit(raw_points)
    result = get_perception_info(raw_points_list, dbscan.labels_, raw_points)
    
    cluster_count = len(result['clusters'])
    outlier_count = len(result['outliers'])
    
    print(f"  ✅ 군집화 완료: {cluster_count}개 군집, {outlier_count}개 이상치")
    
    if cluster_count == 0:
        print("  ⚠️  군집을 찾지 못했습니다. eps, min_samples 값을 조정하세요.")
        return None
    
    # 목적지 좌표 추출
    target_positions = [c['target_center'] for c in result['clusters']]
    
    if show_targets:
        print(f"\n  📍 목적지 좌표 ({len(target_positions)}개):")
        for i, pos in enumerate(target_positions):
            pixel_count = len(result['clusters'][i]['pixels'])
            print(f"     {i+1}. {pos} (크기: {pixel_count}픽셀)")
    
    # ========================================
    # [2단계] 시뮬레이션 준비
    # ========================================
    print("\n[2단계] 시뮬레이션 준비")
    print("-" * 70)
    
    sim = TennisCourtSimulator(map_path=map_file)
    
    # 군집 목적지 로드 및 시각화
    if visualize_clusters:
        sim.load_cluster_targets(result, visualize=True)
    
    # ========================================
    # [3단계] 시뮬레이션 실행
    # ========================================
    print("\n[3단계] 시뮬레이션 실행")
    print("-" * 70)
    print("  🚀 AutoNavSim2D 시뮬레이터를 시작합니다...")
    print("\n" + "=" * 70)
    print("  GUI 사용법:")
    print("  1. 좌클릭 1번: 로봇 시작 위치 설정")
    print("  2. 좌클릭 2번: 목표 위치 설정")
    print("     → 색깔 표시된 군집 중심을 클릭하세요!")
    print("  3. 로봇이 해당 군집으로 이동합니다")
    print("  4. 다시 시작 위치 설정 → 다음 군집 클릭 (반복)")
    print("=" * 70 + "\n")
    
    sim.run()
    
    print("\n✅ 파이프라인 완료!")
    
    return result


# ============================================================
# 직접 실행 시
# ============================================================

if __name__ == "__main__":
    # ========================================
    # 설정
    # ========================================
    
    MAP_FILE = 'maps/snow_removal_area_multi3.pkl'  # 맵 파일 경로
    EPS = 10                # 군집화 반경
    MIN_SAMPLES = 5         # 최소 샘플 수
    VISUALIZE = True        # 군집 시각화 여부
    
    # ========================================
    # 실행
    # ========================================
    
    run_full_pipeline(
        map_file=MAP_FILE,
        eps=EPS,
        min_samples=MIN_SAMPLES,
        visualize_clusters=VISUALIZE,
        show_targets=True
    )
