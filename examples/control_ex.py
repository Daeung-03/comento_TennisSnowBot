"""제어 팀원 테스트 예제"""
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from main import TennisCourtSimulator
from src.control.planner import custom_path_planner, custom_motion_planner

# 시뮬레이터 초기화
sim = TennisCourtSimulator()

# 커스텀 플래너 등록
sim.set_path_planner(custom_path_planner)
sim.set_motion_planner(custom_motion_planner)

# 실행
sim.run()
