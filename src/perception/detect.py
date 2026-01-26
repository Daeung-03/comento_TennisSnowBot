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


def apply_kmeans_clustering(nav, snow_pixels, area_name="Unknown", color_offset=0):
    """
    2. 군집화 함수
    리턴값: 각 군집의 중심 좌표 리스트 [[r, c], [r, c], ...] (모두 정수형)
    """
    int_centers = []

    if len(snow_pixels) > 5:
        data = np.array(snow_pixels)
        num_clusters = max(1, len(snow_pixels) // 22)

        kmeans = KMeans(n_clusters=num_clusters, n_init=10, random_state=42).fit(data)
        labels = kmeans.labels_

        # 소수점 제거 후 정수 리스트로 변환
        raw_centers = kmeans.cluster_centers_
        for center in raw_centers:
            # 반올림 후 정수형 변환하여 리스트에 추가
            int_centers.append([int(round(center[0])), int(round(center[1]))])

        for idx, coord in enumerate(snow_pixels):
            r, c = coord
            label = labels[idx]
            nav.map_val[r][c][1] = VIVID_COLORS[(label + color_offset) % len(VIVID_COLORS)]

    print(f"\n[Step 2: apply_kmeans_clustering ({area_name}) 결과]")
    print(f" - 정수화된 중심점 리스트: {int_centers}")

    return int_centers


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
