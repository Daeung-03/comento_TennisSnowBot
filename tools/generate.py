"""테니스장 맵 생성 도구"""
from autonavsim2d.autonavsim2d import AutoNavSim2D

config = {
    "show_frame": True,
    "show_grid": False,
    "map": None
}

# 맵 생성 모드 실행
nav = AutoNavSim2D(
    custom_planner='default',
    custom_motion_planner='default',
    window='map_gen',
    config=config
)

print("맵 생성 모드 실행")
print("1. 좌클릭+드래그로 장애물 그리기")
print("2. 우클릭으로 장애물 삭제")
print("3. 'Save Map' 버튼 클릭")
print("4. 파일명: tennis_court.pkl")

nav.run()
