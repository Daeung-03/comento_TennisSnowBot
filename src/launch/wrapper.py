"""AutoNavSim2D 래퍼 클래스"""
from autonavsim2d.autonavsim2d import AutoNavSim2D
from autonavsim2d.utils.utils import RED, GREEN, BLUE, ORANGE


class TennisCourtSimulator:
    """테니스장 제설 로봇 시뮬레이터"""
    
    def __init__(self, map_path=None):
        """
        시뮬레이터 초기화
        
        Args:
            map_path: 맵 파일 경로 (.pkl), None이면 빈 맵
        """
        self.config = {
            "show_frame": True,
            "show_grid": False,
            "map": map_path
        }
        self.custom_planner = 'default'
        self.custom_motion_planner = 'default'
        self.nav = None  # AutoNavSim2D 객체 저장용
        self.cluster_targets = []  # 군집 목적지 저장
    
    def set_path_planner(self, planner_func):
        """커스텀 경로 계획 함수 등록"""
        self.custom_planner = planner_func
        print(f"경로 계획 함수 등록: {planner_func.__name__}")
    
    def set_motion_planner(self, motion_func):
        """커스텀 모션 제어 함수 등록"""
        self.custom_motion_planner = motion_func
        print(f"모션 제어 함수 등록: {motion_func.__name__}")
    
    def load_cluster_targets(self, cluster_data, visualize=True):
        """
        군집 목적지 좌표를 로드하고 맵에 표시
        
        Args:
            cluster_data: get_perception_info() 결과 딕셔너리
            visualize: True면 맵에 군집 시각화
        """
        self.cluster_targets = [c['target_center'] for c in cluster_data['clusters']]
        
        if visualize and len(self.cluster_targets) > 0:
            # AutoNavSim2D 객체 생성 (맵 로드용)
            self.nav = AutoNavSim2D(
                custom_planner=self.custom_planner,
                custom_motion_planner=self.custom_motion_planner,
                window='amr',
                config=self.config
            )
            
            # 각 군집을 색상으로 구분하여 표시
            colors = [RED, GREEN, BLUE, ORANGE]
            
            print(f"\n  🎯 군집 목적지를 맵에 표시합니다...")
            for i, (r, c) in enumerate(self.cluster_targets):
                color = colors[i % len(colors)]
                
                # 군집 중심에 3x3 마커 표시
                for dr in [-1, 0, 1]:
                    for dc in [-1, 0, 1]:
                        try:
                            self.nav.map_val[r + dr][c + dc][1] = color
                        except IndexError:
                            pass
                
                print(f"     목적지 {i+1}: {[r, c]} → {color} 색상")
            
            print(f"  ✅ {len(self.cluster_targets)}개 목적지 표시 완료!")
    
    def run(self):
        """시뮬레이션 실행"""
        print("\n시뮬레이션 시작...")
        print(f"맵: {self.config['map'] or '새 맵'}")
        
        # 군집 로드가 되어있으면 기존 nav 사용
        if self.nav is not None:
            print("  (군집 목적지가 표시된 맵 사용)")
            self.nav.run()
        else:
            # 일반 실행
            nav = AutoNavSim2D(
                custom_planner=self.custom_planner,
                custom_motion_planner=self.custom_motion_planner,
                window='amr',
                config=self.config
            )
            nav.run()
