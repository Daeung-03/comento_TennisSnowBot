"""
wrapper.py - 테니스장 제설 시뮬레이터 통합 래퍼
"""

import os
import sys
import pickle
from autonavsim2d.autonavsim2d import AutoNavSim2D

# 프로젝트 루트를 경로에 추가
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from src.perception.detect import detect_snow_regions
from src.control.planner import create_snow_removal_planners


class SnowRemovalSimulator:
    """
    테니스장 제설 시뮬레이터 통합
    
    Attributes:
        map_path (str): 맵 파일 경로
        snow_clusters (list): 감지된 눈 영역 리스트
        custom_path_planner (func): 경로 계획 함수
        custom_motion_planner (func): 모션 제어 함수
        simulator (AutoNavSim2D): 시뮬레이터 인스턴스
    """
    
    def __init__(self, map_path='maps/TennisCourt_Snow.pkl', show_frame=True, show_grid=True):
        """
        초기화 및 설정
        
        Parameters:
            map_path: 로드할 맵 파일 경로 (.pkl)
            show_frame: 로봇 좌표계(Frame) 표시 여부
            show_grid: 맵 그리드 표시 여부
        """
        self.map_path = map_path
        self.show_frame = show_frame
        self.show_grid = show_grid
        
        # 변수 초기화
        self.map_data = None
        self.snow_clusters = []
        self.custom_path_planner = None
        self.custom_motion_planner = None
        self.simulator = None
        
        print("=" * 60)
        print("🎾 테니스장 제설 시뮬레이터 초기화")
        print("=" * 60)
    
    def load_map_and_detect_snow(self):
        """
        [Step 1] 맵 로드 및 눈 영역 감지 (Perception)
        
        detect.py 모듈을 호출하여 눈 클러스터 정보를 가져옵니다.
        
        Returns:
            list: 감지된 눈 클러스터 리스트
        """
        print(f"\n[Step 1] 맵 로드 및 클러스터 감지")
        print(f"   - 맵 경로: {self.map_path}")
        
        # 파일 존재 확인
        if not os.path.exists(self.map_path):
            print(f"\n❌ 에러: 맵 파일을 찾을 수 없습니다: {self.map_path}")
            print(f"\n💡 해결 방법:")
            print(f"   python map.py  # 맵 생성")
            sys.exit(1)
        
        # 눈 영역 감지
        result = detect_snow_regions(self.map_path)
        
        if result is None:
            print("❌ 에러: 눈 영역 감지 실패")
            sys.exit(1)
        
        self.map_data = result['map_val']
        self.snow_clusters = result['all_boxes']
        
        # 클러스터 정보 출력
        print(f"\n📋 감지된 제설 구역:")
        for idx, cluster in enumerate(self.snow_clusters, 1):
            (r1, c1), (r2, c2) = cluster
            width = c2 - c1 + 1
            height = r2 - r1 + 1
            area = width * height
            print(f"   {idx}. 위치: ({r1},{c1})-({r2},{c2}) | 크기: {width}x{height} ({area}px)")
        
        return self.snow_clusters
    
    def create_planners(self):
        """
        [Step 2] 경로 및 모션 플래너 생성 (Control)
        
        planner.py의 팩토리 함수를 호출하여 
        현재 눈 상황에 맞는 전용 플래너 함수들을 생성합니다.
        
        Returns:
            tuple: (path_planner, motion_planner)
        """
        print(f"\n[Step 2] Custom Planner 생성")
        
        if not self.snow_clusters:
            print("⚠️ 경고: 눈 클러스터가 없습니다. 먼저 load_map_and_detect_snow()를 실행하세요.")
            return None
        
        # Closure 패턴으로 planner 생성(factory 함수 호출)
        self.custom_path_planner, self.custom_motion_planner = create_snow_removal_planners(
            self.snow_clusters
        )
        
        print(f"✅ Custom Planner 생성 완료")
        
        return self.custom_path_planner, self.custom_motion_planner
    
    def initialize_simulator(self):
        """
        [Step 3] AutoNavSim2D 시뮬레이터 초기화
        
        준비된 맵 데이터와 플래너를 시뮬레이터에 주입합니다.
        
        Returns:
            AutoNavSim2D: 초기화된 시뮬레이터 객체
        """
        print(f"\n[Step 3] AutoNavSim2D 초기화")
        
        # 사전 조건 확인
        if self.custom_path_planner is None or self.custom_motion_planner is None:
            print("⚠️ 경고: Planner가 생성되지 않았습니다. 먼저 create_planners()를 실행하세요.")
            return None
        
        if self.map_data is None:
            print("⚠️ 경고: 맵 데이터가 없습니다. 먼저 load_map_and_detect_snow()를 실행하세요.")
            return None
        
        # 맵 데이터를 임시 파일로 저장(시뮬레이터)
        temp_map_path = 'maps/temp_Snow_map.pkl'
        os.makedirs('maps', exist_ok=True)
        with open(temp_map_path, 'wb') as f:
            pickle.dump(self.map_data, f)
        
        # AutoNavSim2D 초기화
        config = {
            'show_frame': self.show_frame,
            'show_grid': self.show_grid,
            'map': temp_map_path
        }
        
        try:
            # 시뮬레이터 인스턴스 생성
            self.simulator = AutoNavSim2D(
                custom_planner=self.custom_path_planner,
                custom_motion_planner=self.custom_motion_planner,
                window='amr',
                config=config
            )
            
            # custom_motion_planner가 None이 아닐 때 속성이 설정되지 않는 문제 해결
            if not hasattr(self.simulator, 'custom_motion_planner'):
                print(" 💡 Workaround: Motion Planner 속성 강제 설정")
                # dev_custom_motion_planner를 custom_motion_planner로 복사
                if hasattr(self.simulator, 'dev_custom_motion_planner'):
                    self.simulator.custom_motion_planner = self.simulator.dev_custom_motion_planner
                else:
                    # 직접 설정
                    self.simulator.custom_motion_planner = self.custom_motion_planner
                    self.simulator.dev_custom_motion_planner = self.custom_motion_planner
            
            print(f"✅ AutoNavSim2D 초기화 완료")
            print(f"   - Window: amr (Autonomous Mobile Robot)")
            print(f"   - Show Frame: {self.show_frame}")
            print(f"   - Show Grid: {self.show_grid}")
            
        except Exception as e:
            print(f"❌ AutoNavSim2D 초기화 실패: {e}")
            import traceback
            traceback.print_exc()
            return None
        
        return self.simulator
    
    def run(self):
        """
        [Step 4] 시뮬레이션 루프 실행
        """
        print(f"\n[Step 4] 시뮬레이션 시작")
        print("=" * 60)
        
        if self.simulator is None:
            print("❌ 에러: 시뮬레이터가 초기화되지 않았습니다.")
            print("   initialize_simulator()를 먼저 실행하세요.")
            return
        
        print("\n🎮 시뮬레이션 사용법:")
        print("   1. 맵에서 로봇 시작 위치를 클릭하세요 (빨간색)")
        print("   2. 목표 위치를 클릭하세요 (녹색) - 제설에서는 의미 없음")
        print("   3. 'Plan Path' 버튼을 클릭하여 경로를 생성하세요")
        print("      → 모든 클러스터를 순회하는 제설 경로가 생성됩니다")
        print("   4. 'Navigate' 버튼을 클릭하여 로봇을 움직이세요")
        print("      → 각 클러스터를 boustrophedon(ㄹ) 패턴으로 완전히 커버합니다")
        print("   5. 'Reset' 버튼으로 재시작할 수 있습니다")
        print("\n💡 팁:")
        print("   - 시작 위치는 아무 곳이나 가능합니다")
        print("   - 로봇이 가장 가까운 클러스터부터 순차적으로 방문합니다")
        print("   - 대시보드에서 실시간 위치와 진행상황을 확인할 수 있습니다")
        print("\n" + "=" * 60)
        
        try:
            self.simulator.run()
        except KeyboardInterrupt:
            print("\n\n⚠️ 사용자에 의해 중단되었습니다.")
        except Exception as e:
            print(f"\n❌ 에러 발생: {e}")
            import traceback
            traceback.print_exc()
    
    def quick_start(self):
        """전체 초기화 및 실행을 한 번에 수행"""
        print("\n🚀 Quick Start 모드\n")
        
        try:
            # 1. 맵 로드 및 눈 감지
            self.load_map_and_detect_snow()
            
            # 2. Planner 생성
            self.create_planners()
            
            # 3. 시뮬레이터 초기화
            self.initialize_simulator()
            
            # 4. 실행
            self.run()
            
        except Exception as e:
            print(f"\n❌ Quick Start 실패: {e}")
            import traceback
            traceback.print_exc()

