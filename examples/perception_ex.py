"""
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