"""
examples/control_ex.py - Control 모듈 모니터링
"""
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, project_root)

try:
    from src.launch.wrapper import SnowRemovalSimulator
    from src.control.planner import create_snow_removal_planners
except ImportError:
    print("❌ 오류: 모듈을 찾을 수 없습니다.")
    sys.exit(1)

def monitor_control():
    map_path = os.path.join(project_root, 'maps', 'TennisCourt_Snow.pkl')
    if not os.path.exists(map_path):
        print(f"❌ 맵 파일 없음")
        return

    print("=" * 60)
    print("🤖 Control Module Monitoring (Debug Mode)")
    print("=" * 60)

    sim = SnowRemovalSimulator(map_path=map_path, show_frame=True, show_grid=True)

    sim.load_map_and_detect_snow()

    print(">>> 2. Create Planners (Manual Injection)...")
    
    sim.custom_path_planner, sim.custom_motion_planner = create_snow_removal_planners(
        sim.snow_clusters, 
        debug_mode=True 
    )
    
    # 확인용 출력
    if sim.custom_path_planner:
        print("    -> Planner Injected Successfully.")
    else:
        print("    -> ❌ Planner Injection Failed.")

    print(">>> 3. Initialize Simulator...")
    sim.initialize_simulator()

    print(">>> 4. Run Simulation...")
    try:
        sim.run()
    except KeyboardInterrupt:
        print("\n⚠️ 중단됨")
    except Exception as e:
        print(f"\n❌ 에러: {e}")

if __name__ == "__main__":
    monitor_control()