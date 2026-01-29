"""
map_view.py - 초간단 맵 뷰어
"""
from autonavsim2d.autonavsim2d import AutoNavSim2D

AutoNavSim2D(
    window='amr',
    config={
        "show_frame": False,
        "show_grid": False,
        "map": 'maps/TennisCourt_Snow.pkl'
    }
).run()
