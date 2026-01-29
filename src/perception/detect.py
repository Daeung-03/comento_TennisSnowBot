"""
detect.py - 눈 영역 감지 및 군집화
"""

import numpy as np
import pickle
import os
from sklearn.cluster import DBSCAN
from autonavsim2d.autonavsim2d import AutoNavSim2D

# 각 군집 시각화 색상 (녹색 계열 제외)
VIVID_COLORS = [
    (255, 0, 0),      # RED
    (0, 0, 255),      # BLUE
    (255, 255, 0),    # YELLOW
    (255, 0, 255),    # MAGENTA
    (0, 255, 255),    # CYAN
    (255, 128, 0),    # ORANGE
    (128, 0, 255),    # PURPLE
    (255, 255, 255)   # WHITE
]

# 보호할 색상 (덮어쓰지 않음)
PROTECTED_COLORS = [
    (255, 0, 0),      # RED (시작점)
    (0, 255, 0),      # GREEN (목표점)
    (255, 165, 0)     # ORANGE (경로)
]

# 코트 구분 색
BACKGROUND_COLORS = [
    (0 ,0 ,0),          #BLACK
    (255, 255, 255),    #WHITE        
    (128, 128, 128),    #GREY
    (0, 255, 0)         #GREEN
]


def load_map_and_extract_snow(map_path: str):
    """
    맵 파일(.pkl) 로드 및 눈 픽셀 추출

    Parameters:
        map_path: 군집화가 돠지 않은 기본 맵
    
    Returns:
        tuple: (상단_눈_픽셀, 하단_눈_픽셀, 맵_데이터)
    """
    print(f"\n[Step 1] 맵 로드: {map_path}")
    
    if not os.path.exists(map_path):    #맵 파일이 존재하지 않음
        return [], [], None
    
    with open(map_path, 'rb') as f:
        map_val = pickle.load(f)
    
    snow_top = []   #눈 픽셀 상단
    snow_bottom = []    #눈 픽셀 하단
    
    rows = len(map_val)
    cols = len(map_val[0])
    net_row = rows // 2  # 네트 위치
    
    # 코트에서 코트 기본 정보 외의 픽셀 추출(눈)
    for r in range(rows):
        for c in range(cols):
            try:
                color = map_val[r][c][1]
                if color not in BACKGROUND_COLORS:
                    if r < net_row:
                        snow_top.append([r, c])
                    else:
                        snow_bottom.append([r, c])
            except: 
                pass
    
    return snow_top, snow_bottom, map_val


def apply_clustering(map_val, snow_pixels, area_name="Unknown", color_offset=0):
    """
    DBSCAN 군집화 수행 및 Bounding BOX(작업 구역)생성
    
    Parameters:
        map_val: 맵 데이터
        snow_pixels: 눈 픽셀 리스트
        area_name: 영역 이름
        color_offset: 색상 오프셋
    
    Returns:
        list: Bounding box 리스트 [((r1,c1), (r2,c2)), ...]
    """
    bounding_boxes = []
    
    if len(snow_pixels) < 5:
        return bounding_boxes
    
    data = np.array(snow_pixels)
    
    # DBSCAN 수행
    dbscan = DBSCAN(eps=8, min_samples=3).fit(data) #민감도 설정 = eps:거리, min_samples:최소 점 개수
    labels = dbscan.labels_
    unique_labels = set(labels)
    unique_labels.discard(-1)  #노이즈 제거
    
    # Bounding Box 계산 및 시각화
    for label in unique_labels:
        cluster_points = data[labels == label]
        
        if len(cluster_points) == 0:
            continue
        
        # 좌상단, 우하단 탐색
        r_min, c_min = np.min(cluster_points, axis=0)
        r_max, c_max = np.max(cluster_points, axis=0)
        
        bbox = ((int(r_min), int(c_min)), (int(r_max), int(c_max)))
        bounding_boxes.append(bbox)
        
        # 색상 칠하기
        color_idx = list(unique_labels).index(label)
        color = VIVID_COLORS[(color_idx + color_offset) % len(VIVID_COLORS)]
        
        for pr, pc in cluster_points:
            try:
                current_color = map_val[pr][pc][1]
                if current_color not in PROTECTED_COLORS:
                    map_val[pr][pc][1] = color
            except:
                pass
    
    return bounding_boxes


def detect_snow_regions(map_path):
    """
    Main Interface
    Parameters:
        map_path: 군집화가 돠지 않은 기본 맵
    
    Returns:
        dict: {
            'map_val': 맵 데이터,
            'top_boxes': 상단 박스,
            'bottom_boxes': 하단 박스,
            'all_boxes': 전체 박스
        }
    """
    # 맵 로드 및 눈 추출
    top_pixels, bottom_pixels, map_val = load_map_and_extract_snow(map_path)
    
    if map_val is None:
        return None
    
    # 군집화
    top_boxes = apply_clustering(map_val, top_pixels, "상단 코트", color_offset=0)
    bottom_boxes = apply_clustering(map_val, bottom_pixels, "하단 코트", color_offset=4)
    
    all_boxes = top_boxes + bottom_boxes
    
    return {
        'map_val': map_val,
        'top_boxes': top_boxes,
        'bottom_boxes': bottom_boxes,
        'all_boxes': all_boxes
    }
