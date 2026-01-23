import pickle
import numpy as np
import random
import pygame
from sklearn.cluster import DBSCAN
from autonavsim2d.autonavsim2d import AutoNavSim2D
from autonavsim2d.utils.utils import WHITE, BLACK, GREEN, BLUE, RED, GREY, ORANGE

# --- 색상 및 상태 정의 ---
CAT_PINK = (255, 0, 255)
NET_CYAN = (0, 255, 255)

COLORS = {
    0: GREEN, 1: BLUE, 2: NET_CYAN, 3: CAT_PINK, 4: WHITE, 5: GREY
}


def generate_court_coords():
    state5 = []
    # 배경 확장 (20,30 ~ 170,210)
    for r in range(20, 171):
        for c in range(30, 211):
            if (60 <= r <= 140 and 60 <= c <= 100) or (60 <= r <= 140 and 140 <= c <= 180):
                continue
            state5.append([r, c])

    # 초록코트 (우측)
    state0 = [[r, c] for r in range(60, 101) for c in range(140, 181)]

    # 흰눈코트 (제설 구역)
    state1 = []
    for r in range(60, 141):
        for c in range(60, 181):
            if 60 <= r <= 140 and 100 < c < 140: continue  # 코트 사이 공간
            if 60 <= r <= 100 and 140 <= c <= 180: continue  # 초록코트 제외
            if r == 110 and c == 80: continue  # 고양이 자리
            state1.append([r, c])

    # 네트 (중앙 분리대 역할)
    state2 = [[100, c] for c in range(60, 181) if not (100 < c < 140)]

    return state0, state1, state2, state5


def get_nearest_free_space(matrix, target_r, target_c):
    mat = np.array(matrix)
    rows, cols = mat.shape
    if mat[target_r][target_c] == 1: return target_r, target_c
    for dist in range(1, 15):
        for dr in range(-dist, dist + 1):
            for dc in range(-dist, dist + 1):
                nr, nc = target_r + dr, target_c + dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    if mat[nr][nc] == 1: return nr, nc
    return target_r, target_c


def run_snow_removal_sim():
    s0, s1, s2, s5 = generate_court_coords()
    config = {"show_frame": True, "show_grid": True, "map": 'blank_map.pkl'}
    nav = AutoNavSim2D(custom_planner='default', custom_motion_planner='default', window='amr', config=config)

    for r, c in s5: nav.map_val[r][c][1] = COLORS[5]
    for r, c in s0: nav.map_val[r][c][1] = COLORS[0]
    for r, c in s1: nav.map_val[r][c][1] = COLORS[1]
    for r, c in s2: nav.map_val[r][c][1] = COLORS[2]

    cat_r, cat_c = 110, 80
    nav.map_val[cat_r][cat_c][1] = COLORS[3]

    temp_matrix = nav.generate_grid_matrix(nav.map_val)

    # --- [군집화 개선 로직] ---
    raw_points = np.array(s1)

    # 장애물(네트) 기준 공간 분리를 위해 특징 벡터 확장
    # R좌표(세로)가 100(네트)보다 큰지 작은지 여부를 특징으로 추가 (가중치 100 부여)
    features = []
    for p in raw_points:
        side_of_net = 1 if p[0] > 100 else 0
        features.append([p[0], p[1], side_of_net * 100])  # 공간적 단절 부여

    features = np.array(features)
    dbscan = DBSCAN(eps=7, min_samples=15).fit(features)

    print("\n" + "=" * 50)
    print(f"{'Cluster ID':^12} | {'Size':^8} | {'Center (Row, Col)':^18}")
    print("-" * 50)

    unique_labels = set(dbscan.labels_)
    for label in sorted(unique_labels):
        if label == -1: continue  # 노이즈 패스

        indices = np.where(dbscan.labels_ == label)[0]
        cluster_points = raw_points[indices]

        # 중앙값 계산
        center = cluster_points.mean(axis=0)
        tr, tc = round(center[0]), round(center[1])

        # 목적지 보정 및 출력
        final_tr, final_tc = get_nearest_free_space(temp_matrix, tr, tc)

        print(f"{label:^12} | {len(indices):^8} | ({final_tr:>3}, {final_tc:>3})")

        # 맵에 목적지 표시 (빨간 점)
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                if 0 <= final_tr + dr < 200 and 0 <= final_tc + dc < 250:
                    nav.map_val[final_tr + dr][final_tc + dc][1] = RED

    print("=" * 50)
    # -----------------------

    final_matrix = nav.generate_grid_matrix(nav.map_val)
    nav.matrix = final_matrix
    if not isinstance(nav.planner, str): nav.planner.matrix = final_matrix

    nav.run()