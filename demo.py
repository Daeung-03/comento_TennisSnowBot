from autonavsim2d.autonavsim2d import AutoNavSim2D

# 기본 설정
config = {
    "show_frame": True,
    "show_grid": False,
    "map": None  # 또는 "maps/tennis_court.pkl"
}

# 기본 Dijkstra 알고리즘으로 실행
nav = AutoNavSim2D(
    custom_planner='default',
    custom_motion_planner='default',
    window='amr',
    config=config
)

nav.run()
