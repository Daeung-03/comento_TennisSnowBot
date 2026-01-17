"""제어 팀원 테스트 예제"""
from src.integration.sim_wrapper import TennisCourtSimulator
from src.control.path_planner import custom_path_planner, custom_motion_planner

# 시뮬레이터 초기화
sim = TennisCourtSimulator()

# 커스텀 플래너 등록
sim.set_path_planner(custom_path_planner)
sim.set_motion_planner(custom_motion_planner)

# 실행
sim.run()
