"""
wrapper.py
테니스장 제설 시뮬레이터 래퍼 클래스
AutoNavSim2D, detect1.py, Planner.py를 통합
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
    테니스장 제설 시뮬레이터
    
    주요 기능:
    1. 맵 로드 및 눈 영역 감지 (DBSCAN)
    2. 제설 경로 계획 및 모션 제어 생성
    3. AutoNavSim2D 시뮬레이션 실행
    """
    
    def __init__(self, map_path='maps/TennisCourt_Clustered.pkl', show_frame=True, show_grid=True):
        """
        Parameters:
            map_path (str): 테니스 코트 맵 파일 경로
            show_frame (bool): 로봇 프레임 표시 여부
            show_grid (bool): 그리드 표시 여부
        """
        self.map_path = map_path
        self.show_frame = show_frame
        self.show_grid = show_grid
        
        # 초기화
        self.map_data = None
        self.snow_clusters = []
        self.custom_path_planner = None
        self.custom_motion_planner = None
        self.simulator = None
        
        print("=" * 60)
        print("🎾 테니스장 제설 시뮬레이터 초기화")
        print("=" * 60)
    
    def load_map_and_detect_snow(self):
        """맵 로드 및 눈 영역 감지"""
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
        
        print(f"\n✅ 맵 로드 완료")
        print(f"   - 상단 클러스터: {len(result['top_boxes'])}개")
        print(f"   - 하단 클러스터: {len(result['bottom_boxes'])}개")
        print(f"   - 전체 제설 영역: {len(self.snow_clusters)}개")
        
        # 클러스터 정보 출력
        print(f"\n📋 감지된 클러스터:")
        for idx, cluster in enumerate(self.snow_clusters, 1):
            (r1, c1), (r2, c2) = cluster
            width = c2 - c1 + 1
            height = r2 - r1 + 1
            area = width * height
            print(f"   {idx}. 위치: ({r1},{c1})-({r2},{c2}) | 크기: {width}x{height} ({area}px)")
        
        return self.snow_clusters
    
    def create_planners(self):
        """제설 경로 계획 및 모션 제어 함수 생성"""
        print(f"\n[Step 2] Custom Planner 생성")
        
        if not self.snow_clusters:
            print("⚠️ 경고: 눈 클러스터가 없습니다. 먼저 load_map_and_detect_snow()를 실행하세요.")
            return None
        
        # Closure 패턴으로 planner 생성
        self.custom_path_planner, self.custom_motion_planner = create_snow_removal_planners(
            self.snow_clusters
        )
        
        print(f"✅ Custom Planner 생성 완료")
        print(f"   - Path Planner: {self.custom_path_planner.__name__}")
        print(f"   - Motion Planner: {self.custom_motion_planner.__name__}")
        print(f"   - Captured Clusters: {len(self.snow_clusters)}개")
        
        return self.custom_path_planner, self.custom_motion_planner
    
    def initialize_simulator(self):
        """AutoNavSim2D 시뮬레이터 초기화"""
        print(f"\n[Step 3] AutoNavSim2D 초기화")
        
        if self.custom_path_planner is None or self.custom_motion_planner is None:
            print("⚠️ 경고: Planner가 생성되지 않았습니다. 먼저 create_planners()를 실행하세요.")
            return None
        
        if self.map_data is None:
            print("⚠️ 경고: 맵 데이터가 없습니다. 먼저 load_map_and_detect_snow()를 실행하세요.")
            return None
        
        # 맵 데이터를 임시 파일로 저장 (AutoNavSim2D가 파일 경로를 요구하므로)
        temp_map_path = 'maps/temp_clustered_map.pkl'
        os.makedirs('maps', exist_ok=True)
        
        with open(temp_map_path, 'wb') as f:
            pickle.dump(self.map_data, f)
        
        print(f"   - 임시 맵 저장: {temp_map_path}")
        
        # AutoNavSim2D 초기화
        config = {
            'show_frame': self.show_frame,
            'show_grid': self.show_grid,
            'map': temp_map_path
        }
        
        try:
            self.simulator = AutoNavSim2D(
                custom_planner=self.custom_path_planner,
                custom_motion_planner=self.custom_motion_planner,
                window='amr',
                config=config
            )
            
            # ✅ 수정: AutoNavSim2D 버그 우회
            # custom_motion_planner가 None이 아닐 때 속성이 설정되지 않는 문제 해결
            if not hasattr(self.simulator, 'custom_motion_planner'):
                print(f"   ⚠️ AutoNavSim2D 버그 감지: custom_motion_planner 속성 누락")
                print(f"   💡 Workaround 적용 중...")
                # dev_custom_motion_planner를 custom_motion_planner로 복사
                if hasattr(self.simulator, 'dev_custom_motion_planner'):
                    self.simulator.custom_motion_planner = self.simulator.dev_custom_motion_planner
                    print(f"   ✅ 속성 복사 완료")
                else:
                    # 직접 설정
                    self.simulator.custom_motion_planner = self.custom_motion_planner
                    self.simulator.dev_custom_motion_planner = self.custom_motion_planner
                    print(f"   ✅ 속성 직접 설정 완료")
            
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
        """시뮬레이터 실행"""
        print(f"\n[Step 4] 시뮬레이션 시작")
        print("=" * 60)
        
        if self.simulator is None:
            print("❌ 에러: 시뮬레이터가 초기화되지 않았습니다.")
            print("   initialize_simulator()를 먼저 실행하세요.")
            return
        
        print("\n🎮 시뮬레이션 사용법:")
        print("   1. 맵에서 로봇 시작 위치를 클릭하세요 (빨간색)")
        print("   2. 목표 위치를 클릭하세요 (녹색) - 모션에는 의미 없음")
        print("   3. 'Plan Path' 버튼을 클릭하여 경로를 생성하세요")
        print("      → 모든 클러스터를 순회하는 제설 경로가 생성됩니다")
        print("   4. 'Navigate' 버튼을 클릭하여 로봇을 움직이세요")
        print("      → 각 클러스터를 ㄹ 패턴으로 완전히 커버합니다")
        print("   5. 'Reset' 버튼으로 재시작할 수 있습니다")
        print("\n💡 팁:")
        print("   - 시작 위치는 코트 내부 아무 곳이나 가능합니다")
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


# ==================== 편의 함수 ====================

def create_simulator(map_path='maps/TennisCourt_Clustered.pkl', show_frame=True, show_grid=True):
    """
    시뮬레이터 생성 및 초기화 (헬퍼 함수)
    
    Parameters:
        map_path (str): 맵 파일 경로
        show_frame (bool): 로봇 프레임 표시
        show_grid (bool): 그리드 표시
    
    Returns:
        SnowRemovalSimulator: 초기화된 시뮬레이터 객체
    """
    sim = SnowRemovalSimulator(map_path, show_frame, show_grid)
    sim.load_map_and_detect_snow()
    sim.create_planners()
    sim.initialize_simulator()
    return sim


def quick_run(map_path='maps/TennisCourt_Clustered.pkl', show_frame=True, show_grid=True):
    """
    시뮬레이터 생성부터 실행까지 한 번에 수행 (최단 실행)
    
    Parameters:
        map_path (str): 맵 파일 경로
        show_frame (bool): 로봇 프레임 표시
        show_grid (bool): 그리드 표시
    """
    sim = SnowRemovalSimulator(map_path, show_frame, show_grid)
    sim.quick_start()


# ==================== 테스트 코드 ====================

if __name__ == "__main__":
    """
    wrapper.py를 직접 실행하면 테스트 모드로 동작
    """
    import argparse
    
    parser = argparse.ArgumentParser(description='테니스장 제설 시뮬레이터')
    parser.add_argument('--map', type=str, default='maps/TennisCourt_Clustered.pkl',
                        help='맵 파일 경로')
    parser.add_argument('--no-frame', action='store_true',
                        help='로봇 프레임 숨기기')
    parser.add_argument('--no-grid', action='store_true',
                        help='그리드 숨기기')
    parser.add_argument('--step-by-step', action='store_true',
                        help='단계별 실행 모드 (디버깅용)')
    
    args = parser.parse_args()
    
    print("\n" + "=" * 60)
    print("🧪 wrapper.py 테스트 모드")
    print("=" * 60)
    
    if args.step_by_step:
        # 방법 1: 단계별 실행 (디버깅용)
        print("\n📝 단계별 실행 모드\n")
        sim = SnowRemovalSimulator(
            map_path=args.map,
            show_frame=not args.no_frame,
            show_grid=not args.no_grid
        )
        
        sim.load_map_and_detect_snow()
        input("\n⏸️ [Enter]를 누르면 다음 단계로 진행합니다...")
        
        sim.create_planners()
        input("\n⏸️ [Enter]를 누르면 다음 단계로 진행합니다...")
        
        sim.initialize_simulator()
        input("\n⏸️ [Enter]를 누르면 시뮬레이션을 시작합니다...")
        
        sim.run()
    
    else:
        # 방법 2: Quick Start (기본)
        print("\n🚀 Quick Start 모드\n")
        quick_run(
            map_path=args.map,
            show_frame=not args.no_frame,
            show_grid=not args.no_grid
        )
