"""
눈 감지 및 맵 생성 모듈
카메라/센서 데이터에서 눈이 쌓인 영역을 찾아내고, AutoNavSim2D가 이해할 수 있는 맵 형식으로 변환
"""


class SnowDetector:
    """눈 영역 감지 클래스"""
    
    def __init__(self):
        """초기화 - 필요한 모델 로드 등"""
        pass
    
    def detect_snow(self, image):
        """
        이미지에서 눈이 쌓인 영역 감지
        
        Args:
            image: 입력 이미지 (numpy array 등)
        
        Returns:
            snow_mask: 눈 영역 마스크 (numpy array, 0=눈없음, 1=눈있음)
        
        TODO: 인지 팀원이 구현
        """
        raise NotImplementedError("인지 팀원이 구현해야 합니다")
    
    def generate_map(self, snow_mask):
        """
        눈 마스크를 AutoNavSim2D 맵 형식으로 변환
        
        Args:
            snow_mask: 눈 영역 마스크
        
        Returns:
            map_matrix: 2D 리스트 (1=이동가능, 0=장애물)
        
        """
        raise NotImplementedError("인지 팀원이 구현해야 합니다")
