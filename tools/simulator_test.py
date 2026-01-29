"""
simulator_test.py - 시뮬레이터 기본 조작
"""
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

print("\n🎮 시뮬레이션 사용법:")
print("   1. 맵에서 로봇 시작 위치를 클릭하세요")
print("   2. 목표 위치를 클릭하세요")
print("   3. 클릭 + 드래그로 장애물을 생성할 수 있습니다")
print("   4. 'Plan Path' 버튼을 클릭하여 경로를 생성하세요")
print("   5. 'Navigate' 버튼을 클릭하여 로봇을 움직이세요")
print("   6. 'Reset' 버튼으로 재시작할 수 있습니다")

nav.run()
