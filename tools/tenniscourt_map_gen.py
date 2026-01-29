"""
tenniscourt_map_gen.py - 테니스 코트 맵 생성 도구
코트 라인, 네트, 그리고 무작위 눈 영역(Snow Patch)을 포함합니다
"""
import pickle
import os

# AutoNavSim2D 그리드 규격
GRID_HEIGHT = 175  # 872 / 5 (cell_spacing)
GRID_WIDTH = 230   # 1147 / 5
CELL_SIZE = 4
CELL_SPACING = 5

# 색상 정의
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)      # 코트 바닥
BLACK = (0, 0, 0)        # 경계선/네트
GREY = (128, 128, 128)   # 외부 영역
BLUE = (100, 149, 237)   # 눈 영역 (테스트용)
LIGHT_BLUE = (173, 216, 230)

class TennisCourtMapGenerator:
    """테니스 코트 맵 생성기"""
    
    def __init__(self):
        self.grid = []
        self.generate_base_grid()
    
    def generate_base_grid(self):
        """기본 그리드 생성"""
        print(f"🎾 테니스 코트 맵 생성 중... ({GRID_HEIGHT} x {GRID_WIDTH})")
        
        for i in range(GRID_HEIGHT):
            row = []
            for j in range(GRID_WIDTH):
                # 기본은 회색 배경
                cell_color = GREY
                # [rect_info, color, (row, col)]
                cell = [None, cell_color, (i, j)]
                row.append(cell)
            self.grid.append(row)
        
        print(f"✅ 기본 그리드 생성 완료")
    
    def draw_tennis_court(self):
        """
        표준 테니스 코트 라인 그리기
        - 코트 영역, 네트, 서비스 라인, 단식/복식 라인 등을 그림
        - 색상은 GREEN(바닥)과 BLACK(라인) 사용
        
        Returns:
            dict: 코트 주요 좌표 정보
                court_bounds(모서리 좌표 4개), net_row(x좌표), service_lines(x좌표 2개)
        """
        
        # 코트 전체 영역 (복식 기준)
        # 중앙에 배치
        court_width = 100   # 코트 너비 (픽셀 단위)
        court_height = 160  # 코트 길이
        
        start_row = (GRID_HEIGHT - court_height) // 2
        start_col = (GRID_WIDTH - court_width) // 2
        end_row = start_row + court_height
        end_col = start_col + court_width
        
        print(f"🎾 코트 영역: Row[{start_row}:{end_row}], Col[{start_col}:{end_col}]")
        
        # 1. 코트 바닥 (녹색)
        for r in range(start_row, end_row):
            for c in range(start_col, end_col):
                self.grid[r][c][1] = GREEN
        
        # 2. 외곽 경계선 (두께 2)
        self._draw_rectangle(start_row, start_col, end_row, end_col, BLACK, thickness=2)
        
        # 3. 네트 (중앙 가로선)
        net_row = start_row + court_height // 2
        for c in range(start_col, end_col):
            self.grid[net_row][c][1] = BLACK
            if net_row + 1 < GRID_HEIGHT:
                self.grid[net_row + 1][c][1] = BLACK
        
        # 4. 서비스 라인 (네트에서 각각 21피트 = 약 30픽셀)
        service_line_top = net_row - 30
        service_line_bottom = net_row + 30
        
        if service_line_top >= start_row:
            for c in range(start_col, end_col):
                self.grid[service_line_top][c][1] = BLACK
        
        if service_line_bottom < end_row:
            for c in range(start_col, end_col):
                self.grid[service_line_bottom][c][1] = BLACK
        
        # 5. 센터 서비스 라인 (세로선)
        center_col = start_col + court_width // 2
        for r in range(service_line_top, service_line_bottom + 1):
            if start_row <= r < end_row:
                self.grid[r][center_col][1] = BLACK
        
        # 6. 단식 사이드라인 (안쪽 세로선)
        singles_margin = 10  # 단식 코트 너비 차이
        left_singles = start_col + singles_margin
        right_singles = end_col - singles_margin
        
        for r in range(start_row, end_row):
            if 0 <= left_singles < GRID_WIDTH:
                self.grid[r][left_singles][1] = BLACK
            if 0 <= right_singles < GRID_WIDTH:
                self.grid[r][right_singles][1] = BLACK
        
        print(f"✅ 테니스 코트 라인 그리기 완료")
        
        return {
            'court_bounds': (start_row, start_col, end_row, end_col),
            'net_row': net_row,
            'service_lines': (service_line_top, service_line_bottom)
        }

    def add_snow_patches(self, court_info, num_patches=8):
        """
        눈 패치(Snow Patch) 추가
        - 상단/하단 코트의 8개 구역에 무작위로 눈 영역을 생성
        - 실제 제설 테스트를 위한 클러스터 데이터 생성
        """
        start_row, start_col, end_row, end_col = court_info['court_bounds']
        net_row = court_info['net_row']
        
        snow_regions = []
        
        import random
        
        # 코트 크기 계산
        court_width = end_col - start_col
        court_height_top = net_row - start_row
        court_height_bottom = end_row - net_row
        
        print(f"❄️ 눈 패치 {num_patches}개 추가 중...")
        
        # ===== 상단 코트 4개 (좌상, 우상, 좌중, 우중) =====
        top_positions = [
            # 좌상단
            (start_row + 5, start_col + 5),
            # 우상단
            (start_row + 5, start_col + court_width - 25),
            # 좌중앙
            (start_row + court_height_top // 2, start_col + 5),
            # 우중앙
            (start_row + court_height_top // 2, start_col + court_width - 25)
        ]
        
        print(f"\n   상단 코트 눈 배치:")
        for idx, (r, c) in enumerate(top_positions):
            patch_width = random.randint(15, 20)
            patch_height = random.randint(15, 20)
            
            actual_pixels = 0
            for pr in range(r, min(r + patch_height, net_row - 2)):
                for pc in range(c, min(c + patch_width, end_col - 2)):
                    if start_row < pr < net_row and start_col < pc < end_col:
                        if random.random() > 0.2:
                            self.grid[pr][pc][1] = BLUE if random.random() > 0.3 else LIGHT_BLUE
                            actual_pixels += 1
            
            snow_regions.append(((r, c), (r + patch_height, c + patch_width)))
            print(f"      {idx+1}. Row[{r}-{r+patch_height}], Col[{c}-{c+patch_width}] - {actual_pixels}px")
        
        # ===== 하단 코트 4개 (좌상, 우상, 좌하, 우하) =====
        bottom_positions = [
            # 좌상단
            (net_row + 5, start_col + 5),
            # 우상단
            (net_row + 5, start_col + court_width - 25),
            # 좌하단
            (net_row + court_height_bottom // 2, start_col + 5),
            # 우하단
            (net_row + court_height_bottom // 2, start_col + court_width - 25)
        ]
        
        print(f"\n   하단 코트 눈 배치:")
        for idx, (r, c) in enumerate(bottom_positions):
            patch_width = random.randint(15, 20)
            patch_height = random.randint(15, 20)
            
            actual_pixels = 0
            for pr in range(r, min(r + patch_height, end_row - 2)):
                for pc in range(c, min(c + patch_width, end_col - 2)):
                    if net_row < pr < end_row and start_col < pc < end_col:
                        if random.random() > 0.2:
                            self.grid[pr][pc][1] = BLUE if random.random() > 0.3 else LIGHT_BLUE
                            actual_pixels += 1
            
            snow_regions.append(((r, c), (r + patch_height, c + patch_width)))
            print(f"      {idx+1}. Row[{r}-{r+patch_height}], Col[{c}-{c+patch_width}] - {actual_pixels}px")
        
        print(f"\n✅ 눈 패치 추가 완료: {len(snow_regions)}개 영역")
        return snow_regions

    
    def _draw_rectangle(self, r1, c1, r2, c2, color, thickness=1):
        """사각형 테두리 그리기"""
        # 상단
        for t in range(thickness):
            for c in range(c1, c2):
                if 0 <= r1 + t < GRID_HEIGHT:
                    self.grid[r1 + t][c][1] = color
        
        # 하단
        for t in range(thickness):
            for c in range(c1, c2):
                if 0 <= r2 - 1 - t < GRID_HEIGHT:
                    self.grid[r2 - 1 - t][c][1] = color
        
        # 좌측
        for t in range(thickness):
            for r in range(r1, r2):
                if 0 <= c1 + t < GRID_WIDTH:
                    self.grid[r][c1 + t][1] = color
        
        # 우측
        for t in range(thickness):
            for r in range(r1, r2):
                if 0 <= c2 - 1 - t < GRID_WIDTH:
                    self.grid[r][c2 - 1 - t][1] = color
    
    def save_map(self, filename='maps/TennisCourtMap.pkl'):
        """맵을 파일로 저장 - pygame.Rect 객체 포함"""
        import pygame
        pygame.init()  # 중요!
        
        # 디렉토리 생성
        os.makedirs('maps', exist_ok=True)
        
        # AutoNavSim2D 형식으로 변환
        autonavsim_grid = []
        
        for i in range(GRID_HEIGHT):
            row = []
            for j in range(GRID_WIDTH):
                pixel_x = j * CELL_SPACING
                pixel_y = i * CELL_SPACING
                
                # ✅ pygame.Rect 객체 생성 (필수!)
                rect = pygame.rect.Rect(pixel_x, pixel_y, CELL_SIZE, CELL_SIZE)
                
                cell = [rect, self.grid[i][j][1], (i, j)]
                row.append(cell)
            autonavsim_grid.append(row)
        
        with open(filename, 'wb') as f:
            pickle.dump(autonavsim_grid, f)
        
        print(f"💾 맵 저장 완료: {filename}")
        print(f"   - 크기: {GRID_HEIGHT} x {GRID_WIDTH}")
        print(f"   - 파일 경로: {os.path.abspath(filename)}")
    

def generate_tennis_court_map(with_snow=True, num_snow_patches=8):
    """테니스 코트 맵 생성 메인 함수"""
    
    print("=" * 60)
    print("🎾 테니스 코트 맵 생성기")
    print("=" * 60)
    
    generator = TennisCourtMapGenerator()
    
    # 코트 그리기
    court_info = generator.draw_tennis_court()
    
    # 눈 추가 (옵션)
    if with_snow:
        snow_regions = generator.add_snow_patches(court_info, num_patches=num_snow_patches)
        print(f"\n📋 생성된 눈 영역: {len(snow_regions)}개")
        for idx, region in enumerate(snow_regions):
            print(f"   {idx+1}. {region}")
    
    # 저장
    filename = 'maps/TennisCourt_Snow.pkl' if with_snow else 'maps/TennisCourt_Clean.pkl'
    generator.save_map(filename)
    
    print("\n" + "=" * 60)
    print("✅ 맵 생성 완료!")
    print("=" * 60)
    
    return generator.grid


if __name__ == "__main__":
    # 눈이 있는 맵 생성
    generate_tennis_court_map(with_snow=True, num_snow_patches=8)
    
    # 깨끗한 맵도 생성 (비교용)
    generate_tennis_court_map(with_snow=False)
