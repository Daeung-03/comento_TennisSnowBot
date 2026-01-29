"""
examples/launch_ex.py - launch 모듈 기능 테스트 및 실행(래퍼 클래스)
"""

import sys
import os
import argparse

# --- 프로젝트 루트 경로 설정 ---
# src 모듈을 찾기 위해 상위 폴더를 sys.path에 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, project_root)

# 모듈 임포트
try:
    from src.launch.wrapper import SnowRemovalSimulator
except ImportError as e:
    print(f"❌ 임포트 오류: {e}")
    print("프로젝트 루트에서 실행하거나 PYTHONPATH를 확인해주세요.")
    sys.exit(1)

def main():
    """
    CLI 인자 파싱 및 시뮬레이터 실행
    """
    parser = argparse.ArgumentParser(description='테니스장 제설 시뮬레이터 테스트 (Wrapper Test)')
    
    # 맵 파일 경로 옵션
    parser.add_argument('--map', type=str, default='maps/TennisCourt_Snow.pkl',
                        help='사용할 맵 파일 경로 (기본값: maps/TennisCourt_Snow.pkl)')
    
    # 시각화 옵션
    parser.add_argument('--no-frame', action='store_true',
                        help='로봇 좌표계(Frame) 숨기기')
    parser.add_argument('--no-grid', action='store_true',
                        help='맵 그리드 숨기기')
    
    # 실행 모드 옵션
    parser.add_argument('--step-by-step', action='store_true',
                        help='단계별 실행 모드 (엔터키로 진행, 디버깅용)')
    parser.add_argument('--debug', action='store_true',
                        help='디버그 정보 출력 활성화 (Planner 로그 등)')

    args = parser.parse_args()

    # 경로 절대경로로 변환 (실행 위치에 영향받지 않게)
    map_full_path = os.path.join(project_root, args.map)

    print("\n" + "=" * 60)
    print("🧪 wrapper_ex.py 실행 모드")
    print("=" * 60)
    print(f" - 맵 경로: {map_full_path}")
    print(f" - 프레임 표시: {'OFF' if args.no_frame else 'ON'}")
    print(f" - 그리드 표시: {'OFF' if args.no_grid else 'ON'}")
    print(f" - 실행 모드: {'단계별(Step-by-Step)' if args.step_by_step else '자동(Quick Start)'}")
    print("=" * 60 + "\n")

    # 시뮬레이터 인스턴스 생성
    sim = SnowRemovalSimulator(
        map_path=map_full_path,
        show_frame=not args.no_frame,
        show_grid=not args.no_grid
    )

    try:
        if args.step_by_step:
            # [Mode 1] 단계별 실행 (Debugging)
            print("📝 [Step 1] 맵 로드 및 눈 감지")
            sim.load_map_and_detect_snow()
            input("   ⏸️  [Enter]를 누르면 다음으로 진행합니다...")

            print("\n📝 [Step 2] 플래너 생성")
            sim.create_planners()
            input("   ⏸️  [Enter]를 누르면 다음으로 진행합니다...")

            print("\n📝 [Step 3] 시뮬레이터 초기화")
            sim.initialize_simulator()
            input("   ⏸️  [Enter]를 누르면 시뮬레이션을 시작합니다...")

            print("\n📝 [Step 4] 실행 (Ctrl+C로 종료)")
            sim.run()

        else:
            # [Mode 2] Quick Start (Default)
            # SnowRemovalSimulator 클래스 내의 quick_start 메서드 활용
            sim.quick_start()

    except KeyboardInterrupt:
        print("\n\n⚠️ 사용자에 의해 중단되었습니다.")
    except Exception as e:
        print(f"\n❌ 실행 중 치명적 오류 발생:\n {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
