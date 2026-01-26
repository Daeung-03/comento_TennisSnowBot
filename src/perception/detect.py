"""
눈 감지 및 맵 생성 모듈
카메라/센서 데이터에서 눈이 쌓인 영역을 찾아내고, AutoNavSim2D가 이해할 수 있는 맵 형식으로 변환
"""

import pickle
import numpy as np
import random
from sklearn.cluster import DBSCAN
from autonavsim2d.autonavsim2d import AutoNavSim2D
from autonavsim2d.utils.utils import BLACK, GREEN, BLUE, GREY, ORANGE, RED

FILENAME = 'maps/snow_removal_area_multi3.pkl'  #맵을 넣어준다.


# [1] 데이터 로드 로직
class FakeRect:
    def __init__(self, x, y, w, h): self.x, self.y = x, y


class CustomUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if name == '__rect_constructor': return FakeRect
        return super().find_class(module, name)

# ===============================
# 1. 맵의 장애물 좌표 추출
# ===============================
def get_snow_area_list(file_path):
    """
    pkl 파일에서 검은색(눈/장애물)좌표 추출
    
    Returns:
        obstacle_list: 검은색(눈/장애물)좌표 리스트
    """
    with open(file_path, 'rb') as f:
        grid_data = CustomUnpickler(f).load()
    obstacle_list = []
    for row in grid_data:
        for cell in row:
            # cell[1]은 색상 정보, cell[2]는 좌표 정보
            if cell[1] in [(0, 0, 0), [0, 0, 0], "black"]:
                obstacle_list.append(list(cell[2]))
    return obstacle_list

# ===============================
# 2. 장애물 좌표 군집화
# ===============================
def get_perception_info(raw_points_list, labels, raw_points):
    """
    장애물 좌표 군집화 결과를 정리된 딕셔너리로 변환
    
    Args:
        raw_points_list: 원본 좌표 리스트
        labels: DBSCAN 레이블 배열
        raw_points: numpy 배열 형태의 좌표
        
    Returns:
        perception_data: {
            "original_points": 전체 좌표,
            "outliers": 이상치 좌표,
            "clusters": [
                {
                    "cluster_id": 군집 번호,
                    "pixels": 군집 좌표들,
                    "target_center": 군집 중심점 (목적지)
                },
                ...
            ]
        }
    """
    perception_data = {
        "original_points": raw_points_list,
        "outliers": [],
        "clusters": []
    }
    
    unique_labels = set(labels)
    
    for label in sorted(unique_labels):
        indices = np.where(labels == label)[0]
        pts = raw_points[indices].tolist()
        
        # label == -1은 이상치(outlier)
        if label == -1:
            perception_data["outliers"] = pts
            continue
        
        # 군집 중심점 계산
        center = raw_points[indices].mean(axis=0)
        
        perception_data["clusters"].append({
            "cluster_id": int(label),
            "pixels": pts,
            "target_center": [round(center[0]), round(center[1])]
        })
    
    return perception_data

def get_unique_color(label):
    """
    군집 ID에 따른 고유 색상 생성 (시각화용)
    
    Args:
        label: 군집 ID
        
    Returns:
        (R, G, B) 튜플
    """
    if label == -1: 
        return (150, 150, 150)  # 이상치는 회색
    
    fixed_colors = [BLACK, GREEN, BLUE, GREY, ORANGE]
    if 0 <= label < len(fixed_colors): 
        return fixed_colors[label]
    
    # 5개 이상 군집이면 랜덤 색상
    random.seed(int(label))
    return (random.randint(0, 50), random.randint(100, 255), random.randint(100, 255))


# def simulate_perception_info(raw_points_list, labels, raw_points):
#     res = get_perception_info(raw_points_list, labels, raw_points)
#     config = {"show_frame": True, "show_grid": False, "map": FILENAME}
#     nav = AutoNavSim2D(custom_planner='default', custom_motion_planner='default', window='amr', config=config)

#     for c in res['clusters']:
#         color = get_unique_color(c['cluster_id'])
#         for r, c_idx in c['pixels']:
#             nav.map_val[r][c_idx][1] = color
#         tr, tc = c['target_center']
#         nav.map_val[tr][tc][1] = RED

#     print("\n시뮬레이터를 실행합니다...")
#     nav.run()
