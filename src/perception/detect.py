import numpy as np
import random
from sklearn.cluster import KMeans
from autonavsim2d.autonavsim2d import AutoNavSim2D
from autonavsim2d.utils.utils import BLACK, GREEN, GREY

# --- 설정 및 색상 ---
R_SIZE, C_SIZE = 175, 230
VIVID_COLORS = [
    (255, 50, 50), (50, 255, 50), (50, 50, 255), (255, 255, 255),
    (255, 150, 0), (255, 0, 255), (0, 255, 255), (255, 255, 0),
    (0, 128, 0), (128, 0, 128), (0, 0, 128), (150, 75, 0)
]

# TennisCourt_Kmeans.pkl 로 고정된 제설상태맵 쓴다.
# def create_tennis_map(nav):
#     """
#     1. 맵 생성 함수
#     리턴값: [[r, c], [r, c], ...] 형태의 정수 리스트 (상단/하단 분리)
#     """
#     snow_top, snow_bottom = [], []

#     for r in range(R_SIZE):
#         for c in range(C_SIZE):
#             nav.map_val[r][c][1] = GREY

#     for r in range(40, 141):
#         for c in range(40, 191):
#             if not (90 < c < 140):  # 중앙 통로 제외
#                 if r == 90: continue  # 네트 라인 제외
#                 nav.map_val[r][c][1] = GREEN
#                 if random.random() < 0.15:
#                     if r < 90:
#                         snow_top.append([r, c])
#                     else:
#                         snow_bottom.append([r, c])

#     for c in range(40, 191):
#         if not (90 < c < 140):
#             nav.map_val[90][c][1] = BLACK

#     print("\n[Step 1: create_tennis_map 실행 결과]")
#     print(f" - 상단 눈 리스트 (첫 3개): {snow_top[:3]}")
#     print(f" - 하단 눈 리스트 (첫 3개): {snow_bottom[:3]}")

#     return snow_top, snow_bottom



def apply_kmeans_clustering(nav, snow_pixels, area_name="Unknown", color_offset=0, visualize_boxes=True):
    """
    군집화 및 Bounding Box 추출 함수
    """
    bounding_boxes = []
    
    if len(snow_pixels) > 5:
        data = np.array(snow_pixels)
        # 군집 개수 자동 계산 (밀도 기반)
        num_clusters = max(1, len(snow_pixels) // 15)
        
        # K-Means 수행
        kmeans = KMeans(n_clusters=num_clusters, n_init=10, random_state=42).fit(data)
        
        # Bounding Box 계산
        for label in range(num_clusters):
            cluster_points = data[kmeans.labels_ == label]
            if len(cluster_points) == 0: continue
            
            r_min, c_min = np.min(cluster_points, axis=0)
            r_max, c_max = np.max(cluster_points, axis=0)
            
            # (좌상단, 우하단) 저장
            top_left = (int(r_min), int(c_min))
            bottom_right = (int(r_max), int(c_max))
            bounding_boxes.append((top_left, bottom_right))
            
            # 군집 픽셀 색칠
            color = VIVID_COLORS[(label + color_offset) % len(VIVID_COLORS)]
            for pr, pc in cluster_points:
                try: nav.map_val[int(pr)][int(pc)][1] = color
                except: pass
            
            # [NEW] Bounding Box 테두리 그리기
            if visualize_boxes:
                draw_bounding_box(nav, top_left, bottom_right, 
                                color=BOX_BORDER_COLOR, thickness=2)
                
            print(f"   Box {label+1}: {top_left} → {bottom_right} (크기: {r_max-r_min+1}x{c_max-c_min+1})")

    print(f"\n[Step 2] {area_name} 군집화 완료: {len(bounding_boxes)}개 Box 생성")
    return bounding_boxes



def run_simulation():
    """
    3. 실행 함수
    전체 데이터를 통합 관리
    """
    config = {
        "show_frame": True,
        "show_grid": True,
        "map": 'blank_map.pkl',
        "map_size": (R_SIZE, C_SIZE)
    }
    nav = AutoNavSim2D(custom_planner='default', custom_motion_planner='default', window='amr', config=config)

    # 1단계: 맵 생성 (리스트 반환)
    top_pixels, bottom_pixels = create_tennis_map(nav)

    # 2단계: 군집화 및 정수 중심점 추출 (리스트 반환)
    top_centers = apply_kmeans_clustering(nav, top_pixels, area_name="상단", color_offset=0)
    bottom_centers = apply_kmeans_clustering(nav, bottom_pixels, area_name="하단", color_offset=6)

    # 맵 데이터 동기화
    nav.matrix = nav.generate_grid_matrix(nav.map_val)
    if not isinstance(nav.planner, str):
        nav.planner.matrix = nav.matrix

    print("\n[Step 3: 모든 데이터 정수형 리스트 변환 완료]")
    all_centers = top_centers + bottom_centers
    print(f" - 최종 목적지 리스트 (총 {len(all_centers)}개): {all_centers}")

    nav.run()

    return all_centers  # 최종적으로 모든 중심점 리스트 반환


if __name__ == "__main__":
    final_destinations = run_simulation()
