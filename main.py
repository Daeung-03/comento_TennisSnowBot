"""AutoNavSim2D 래퍼 클래스"""
from autonavsim2d.autonavsim2d import AutoNavSim2D


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
    
    def set_path_planner(self, planner_func):
        """
        커스텀 경로 계획 함수 등록
        
        Args:
            planner_func: 경로 계획 함수
        """
        self.custom_planner = planner_func
        print(f"경로 계획 함수 등록: {planner_func.__name__}")
    
    def set_motion_planner(self, motion_func):
        """
        커스텀 모션 제어 함수 등록
        
        Args:
            motion_func: 모션 제어 함수
        """
        self.custom_motion_planner = motion_func
        print(f"모션 제어 함수 등록: {motion_func.__name__}")
    
    def run(self):
        """시뮬레이션 실행"""
        print("시뮬레이션 시작...")
        print(f"맵: {self.config['map'] or '새 맵'}")
        
        nav = AutoNavSim2D(
            custom_planner=self.custom_planner,
            custom_motion_planner=self.custom_motion_planner,
            window='amr',
            config=self.config
        )
        nav.run()
