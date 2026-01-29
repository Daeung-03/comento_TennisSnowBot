"""
<<<<<<< HEAD
눈 감지 및 군집화 실행 파일
"""
import sys
import os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

import numpy as np
from sklearn.cluster import DBSCAN
from src.perception.detect import (
    get_snow_area_list,
    get_perception_info
)


# ============================================================
# 설정
# ============================================================

FILENAME = 'maps/snow_removal_area_multi3.pkl'
EPS = 10
MIN_SAMPLES = 5


# ============================================================
# 메인 실행
# ============================================================

if __name__ == "__main__":
    print("=" * 60)
    print("눈 감지 및 군집화 시작")
    print("=" * 60)
    
    # 1. 눈 좌표 로드
    raw_points_list = get_snow_area_list(FILENAME)
    
    if not raw_points_list:
        print(f"❌ {FILENAME}에서 눈 좌표를 찾을 수 없습니다.")
        exit(1)
    
    print(f"✅ {len(raw_points_list)}개의 눈 좌표를 로드했습니다.")
    
    # 2. DBSCAN 군집화
    raw_points = np.array(raw_points_list)
    dbscan = DBSCAN(eps=EPS, min_samples=MIN_SAMPLES).fit(raw_points)
    
    # 3. 결과 데이터 추출
    result = get_perception_info(raw_points_list, dbscan.labels_, raw_points)
    
    # 4. 제어 팀원에게 전달할 데이터
    print("\n" + "=" * 60)
    print("제어 팀원에게 전달할 데이터")
    print("=" * 60)
    
    print(f"\n📍 목적지 좌표 ({len(result['clusters'])}개):")
    for cluster in result['clusters']:
        print(f"  - Cluster {cluster['cluster_id']}: {cluster['target_center']}")
    
    print("\n✅ 완료!")
=======
examples/perception_ex.py - 눈 감지 알고리즘 테스트 및 모니터링
"""
import os
import sys
import time

# 프로젝트 루트를 경로에 추가
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

try:
    from src.perception.detect import detect_snow_regions # detect.py
except ImportError as e:
    print(f"❌ 임포트 오류: src.perception.detect 모듈을 찾을 수 없습니다.\n({e})")
    print("프로젝트 루트에서 실행하거나 PYTHONPATH를 확인해주세요.")
    sys.exit(1)

def print_separator(title):
    print(f"\n{'='*20} {title} {'='*20}")

def monitor_detection():
    # 1. 맵 파일 설정
    map_filename = 'TennisCourt_Clustered.pkl'
    map_file_path = os.path.join(project_root, 'maps', map_filename)

    print_separator("시스템 초기화")
    print(f"Target Map: {map_file_path}")
    
    if not os.path.exists(map_file_path):
        print("❌ 오류: 맵 파일을 찾을 수 없습니다.")
        return

    # 2. 감지 알고리즘 실행
    start_time = time.time()
    result = detect_snow_regions(map_file_path)
    end_time = time.time()

    # 3. 결과 모니터링
    if result is None:
        print("❌ 감지 실패: 맵 데이터를 로드하지 못했습니다.")
        return

    print_separator("감지 결과 모니터링")
    print(f"⏱  소요 시간: {end_time - start_time:.4f}초")
    print(f"📍 총 감지된 눈 덩어리: {len(result['all_boxes'])}개")
    
    print("\n[상단 코트 영역]")
    if not result['top_boxes']:
        print("  - 감지된 눈 없음")
    for idx, box in enumerate(result['top_boxes']):
        print(f"  #{idx+1} 구역: {box} (Row: {box[0][0]}~{box[1][0]})")

    print("\n[하단 코트 영역]")
    if not result['bottom_boxes']:
        print("  - 감지된 눈 없음")
    for idx, box in enumerate(result['bottom_boxes']):
        print(f"  #{idx+1} 구역: {box} (Row: {box[0][0]}~{box[1][0]})")

    print_separator("작업 큐 생성 예시")
    # 로봇에게 보낼 데이터 형식 미리보기
    task_queue = result['all_boxes']
    print(f"Robot Task Queue ({len(task_queue)} items):")
    print(task_queue)

if __name__ == "__main__":
    monitor_detection()
>>>>>>> feature/control
