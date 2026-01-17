"""기본 시뮬레이터 동작 확인"""
from src.integration.sim_wrapper import TennisCourtSimulator

# 기본 시뮬레이터 실행
sim = TennisCourtSimulator()
sim.run()

# 여기서 GUI로:
# 1. 좌클릭 - 로봇 위치 설정
# 2. 좌클릭 - 목표 위치 설정  
# 3. 좌클릭+드래그 - 장애물 생성
