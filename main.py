"""
main.py - 테니스장 제설 로봇 시뮬레이션 실행 파일
"""

import sys
import os

# 모듈 import 확인
try:
    from src.launch.wrapper import SnowRemovalSimulator
except ImportError as e:
    print(f"❌ 에러: launch 모듈을 찾을 수 없습니다.")
    print(f"   상세: {e}")
    sys.exit(1)

try:
    import pygame
    print("✅ pygame 모듈 로드 성공")
except ImportError:
    print("❌ 에러: pygame이 설치되지 않았습니다.")
    print("   pip install pygame")
    sys.exit(1)

try:
    from autonavsim2d.autonavsim2d import AutoNavSim2D
    print("✅ autonavsim2d 모듈 로드 성공")
except ImportError:
    print("❌ 에러: autonavsim2d가 설치되지 않았습니다.")
    print("   pip install autonavsim2d")
    sys.exit(1)


def main():
    """메인 실행 함수"""
    
    print("\n" + "=" * 60)
    print("🎾 테니스장 제설 로봇 시뮬레이션")
    print("=" * 60)
    
    # 맵 파일 경로 설정
    map_path = 'maps/TennisCourt_Snow.pkl'
    
    # 맵 파일 존재 확인
    if not os.path.exists(map_path):
        print(f"❌ 에러: 맵 파일이 없습니다: {map_path}")
        print(f"   생성된 맵이 다음 위치에 있는지 확인하세요:")
        print(f"      {os.path.abspath(map_path)}")
        sys.exit(1)
    
    # 시뮬레이터 생성 및 실행
    try:
        sim = SnowRemovalSimulator(
            map_path=map_path,
            show_frame=True,   # 로봇 좌표계 표시
            show_grid=True     # 그리드 라인 표시
        )
        sim.quick_start()
    except KeyboardInterrupt:
        print("\n\n⚠️ 사용자에 의해 중단되었습니다.")
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ 예상치 못한 에러 발생:")
        print(f"   {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
