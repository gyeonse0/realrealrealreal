import copy
import random
from types import SimpleNamespace
import vrplib 
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import numpy.random as rnd
from typing import List

from MultiModalState import *

class SolutionPlotter:
    """
    특정 route를 기반으로 location 및 path, cost 정보등을 시각화 해주는 클래스
    """
    def __init__(self, data):
        self.data = data
        self.drone_colors = ['red', 'blue', 'green', 'orange', 'purple']
        self.truck_colors = ['cyan', 'magenta', 'yellow', 'lime', 'pink']
        self.drone_color_index = 0
        self.truck_color_index = 0

    def get_next_drone_color(self):
        color = self.drone_colors[self.drone_color_index]
        self.drone_color_index = (self.drone_color_index + 1) % len(self.drone_colors)
        return color

    def get_next_truck_color(self):
        color = self.truck_colors[self.truck_color_index]
        self.truck_color_index = (self.truck_color_index + 1) % len(self.truck_colors)
        return color

    def plot_current_solution(self, state, name="Multi_Modal Solution"):
        """
        우리가 뽑아낸 routes 딕셔너리 집합과 solution class를 통해서 현재의 cost와 path를 plot 해주는 함수
        """
        fig, ax = plt.subplots(figsize=(12, 10))
        new_state = MultiModalState(state.routes,state.unassigned)
        routes = new_state.routes
        unassigned = new_state.unassigned

        divided_routes = apply_dividing_route_to_routes(routes)

        for route_info in divided_routes:
            vtype = route_info['vtype']
            vid = route_info['vid']
            path = route_info['path']

            if vtype == 'drone':
                color = self.get_next_drone_color()
                path = path if isinstance(path, list) else path[0]
                loc_getter = lambda loc: loc[0] if isinstance(loc, tuple) else loc
                linestyle = '--'
                offset = 0.0001 * (self.drone_color_index + 1) 
                linewidth=2

            elif vtype == 'truck':
                color = self.get_next_truck_color()  # 겹치지 않는 색상 생성
                path = path if isinstance(path, list) else path[0]
                loc_getter = lambda loc: loc[0] if isinstance(loc, tuple) else loc
                linestyle = '-'
                offset = 0
                linewidth=1

            # 경로 그리기
            ax.plot(
                [self.data['node_coord'][loc_getter(loc)][0] for loc in path],
                [self.data['node_coord'][loc_getter(loc)][1]+ offset for loc in path],
                color=color,
                linestyle=linestyle, 
                linewidth=linewidth,
                marker='.',
                label=f'{vtype} {vid}'
            )

            # 방향 화살표 그리기
            for i in range(len(path)-1):
                start = self.data['node_coord'][loc_getter(path[i])]
                end = self.data['node_coord'][loc_getter(path[i+1])]
                ax.annotate("", xy=(end[0], end[1] + offset), xytext=(start[0], start[1] + offset), arrowprops=dict(arrowstyle="->", color=color))


        kwargs = dict(label="Depot", zorder=3, marker="s", s=80)
        ax.scatter(*self.data["node_coord"][self.data["depot"]], c="tab:red", **kwargs)
        for node, (x, y) in self.data["node_coord"].items():
            # 주석의 색상을 조건에 따라 설정
            annotation_color = 'red' if self.data["priority_delivery_time"][node] != 0 else 'black'
            ax.annotate(str(node), (x, y), textcoords="offset points", xytext=(0, 5), ha='center', color=annotation_color)
        new_state.cost_objective()
        total_carrier_cost = new_state.get_carrier_cost()
        total_energy_cost = new_state.get_energy_cost()
        total_refund_cost = new_state.get_refund_cost()
        ax.set_title(f"{name}\nTotal Carrier Cost: {total_carrier_cost} USD\n"
             f"Total Energy Cost: {total_energy_cost} USD\n"
             f"Total Refund Cost: {total_refund_cost} USD")
        ax.set_xlabel("X-coordinate")
        ax.set_ylabel("Y-coordinate")
        existing_handles = ax.get_legend_handles_labels()[0]
        legend_elements = [
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=10, label='Priority Delivery Customer')
        ]
        ax.legend(handles= (existing_handles+legend_elements), frameon=False, ncol=3)
        plt.show()