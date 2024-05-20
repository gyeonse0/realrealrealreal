import copy
import time
import matplotlib.pyplot as plt
import numpy as np
import numpy.random as rnd
from FileReader import *
from SolutionPlotter import *
from RouteInitializer import *
from RouteGenerator import *
from Destroy import *
from Repair import *
from MultiModalState import *


SEED = 1234
rnd_state = np.random.RandomState(None)

vrp_file_path = r'C:\Users\User\OneDrive\바탕 화면\realrealrealreal-main\realrealrealreal-main\real_examples_with_charging\data\multi_modal_data.vrp'

file_reader = FileReader()
data = file_reader.read_vrp_file(vrp_file_path)

destroyer = Destroy()
Rep = Repair()
plotter = SolutionPlotter(data)

initializer = RouteInitializer(data)
initial_truck = initializer.init_truck()

class RouletteWheel:
    def __init__(self):
        self.scores = [20, 5, 1, 0.5] 
        self.decay = 0.8
        self.operators = None
        self.weights = None

    def set_operators(self, operators):
        self.operators = operators
        self.weights = [1.0] * len(operators)

    def select_operators(self):
        total_weight = sum(self.weights)
        probabilities = [weight / total_weight for weight in self.weights]
        selected_index = np.random.choice(len(self.operators), p=probabilities)
        selected_operator = self.operators[selected_index]
        return selected_operator

    def update_weights(self, outcome_rank, selected_operator_idx):
        idx = selected_operator_idx
        score = self.scores[outcome_rank-1]
        self.weights[idx] = self.decay * self.weights[idx] + (1 - self.decay) * score

destroy_operators = [destroyer.random_removal, destroyer.can_drone_removal, destroyer.high_cost_removal, destroyer.priority_customer_removal]
repair_operators = [Rep.random_repair, Rep.drone_first_truck_second, Rep.truck_first_drone_second, Rep.heavy_truck_repair, Rep.light_drone_repair]

destroy_selector = RouletteWheel()
repair_selector = RouletteWheel()

destroy_selector.set_operators(destroy_operators)
repair_selector.set_operators(repair_operators)

destroy_counts = {destroyer.__name__: 0 for destroyer in destroy_operators}
repair_counts = {repairer.__name__: 0 for repairer in repair_operators}
drone_k_opt_count=0

# 초기 설정
iteration_num=100

start_temperature = 100
end_temperature = 0.01
step = 0.1

temperature = start_temperature
current_num=0
outcome_counts = {1: 0, 2: 0, 3: 0, 4: 0}
start_time = time.time()

current_states = []  # 상태를 저장할 리스트
objectives = []  # 목적 함수 값을 저장할 리스트

current_states.append(initial_truck)
objective_value = MultiModalState(initial_truck).cost_objective()
objectives.append(objective_value)

while current_num <= iteration_num:
    if current_num==0:
        selected_destroy_operators = destroy_selector.select_operators()
        selected_repair_operators = repair_selector.select_operators()

        destroyed_state = selected_destroy_operators(initial_truck, rnd_state)
        repaired_state = selected_repair_operators(destroyed_state, rnd_state)

        current_states.append(repaired_state)
        objective_value = MultiModalState(repaired_state).cost_objective()
        objectives.append(objective_value)

        d_idx = destroy_operators.index(selected_destroy_operators)
        r_idx = repair_operators.index(selected_repair_operators)

        destroy_counts[destroy_operators[d_idx].__name__] += 1
        repair_counts[repair_operators[r_idx].__name__] += 1
        current_num+=1

    elif current_num % 10 == 0 and current_num != 0:
        
        k_opt_state = Rep.drone_k_opt(current_states[-1],rnd_state)

        current_states.append(k_opt_state)
        objective_value = MultiModalState(k_opt_state).cost_objective()
        objectives.append(objective_value)
        
        temperature = max(end_temperature, temperature - step)
        current_num+=1
        drone_k_opt_count+=1

    else:
        selected_destroy_operators = destroy_selector.select_operators()
        selected_repair_operators = repair_selector.select_operators()
    
        destroyed_state = selected_destroy_operators(current_states[-1].copy(), rnd_state)
        repaired_state = selected_repair_operators(destroyed_state, rnd_state)

        # 이전 objective 값과 비교하여 수락 여부 결정(accept)
        if np.exp((MultiModalState(current_states[-1]).cost_objective() - MultiModalState(repaired_state).cost_objective()) / temperature) >= rnd.random():
            current_states.append(repaired_state)
            objective_value = MultiModalState(repaired_state).cost_objective()
            objectives.append(objective_value)
            if objective_value == min(objectives):
                outcome = 1
            elif objective_value <= MultiModalState(current_states[-1]).cost_objective():
                outcome = 2
            else: 
                outcome = 3

        else:
            # 이전 상태를 그대로 유지(reject)
            current_states.append(current_states[-1])
            objectives.append(MultiModalState(current_states[-1]).cost_objective())
            outcome = 4

        outcome_counts[outcome] += 1

        d_idx = destroy_operators.index(selected_destroy_operators)
        r_idx = repair_operators.index(selected_repair_operators)

        destroy_selector.update_weights(outcome, d_idx)
        repair_selector.update_weights(outcome, r_idx)

        # 온도 갱신
        temperature = max(end_temperature, temperature - step)

        destroy_counts[destroy_operators[d_idx].__name__] += 1
        repair_counts[repair_operators[r_idx].__name__] += 1
        current_num+=1


min_objective = min(objectives)
min_index = objectives.index(min_objective)
end_time = time.time()
execution_time = end_time - start_time

print("\nBest Objective Value:",MultiModalState(current_states[min_index]).cost_objective())
print("\nBest Solution:",MultiModalState(current_states[min_index]).routes)
print("\nIteration #:",min_index)
pct_diff = 100 * (MultiModalState(current_states[min_index]).cost_objective() - MultiModalState(initial_truck).cost_objective()) / MultiModalState(initial_truck).cost_objective()
print(f"\nThis is {-(pct_diff):.1f}% better than the initial solution, which is {MultiModalState(initial_truck).cost_objective()}.\n")

plotter.plot_current_solution(initial_truck,name="Init Solution(Truck NN)")
plotter.plot_current_solution(current_states[min_index])

plt.figure(figsize=(10, 6))
plt.plot(objectives, label='Current Objective')
plt.plot(np.minimum.accumulate(objectives), color='orange', linestyle='-', label='Best Objective')

plt.title('Progress of Objective Value')
plt.xlabel('Iteration(#)')
plt.ylabel('Objective Value(USD)')
plt.grid(True)
plt.legend()
plt.show()

outcome_messages = {
    1: "The candidate solution is a new global best.(1)",
    2: "The candidate solution is better than the current solution, but not a global best.(2)",
    3: "The candidate solution is accepted.(3)",
    4: "The candidate solution is rejected.(4)"
}
for outcome, count in outcome_counts.items():
    print(f"{outcome_messages[outcome]}: {count} times")


print("\nDestroy Operator Counts(#):")
for name, count in destroy_counts.items():
    print(f"{name}: {count}")
print("\nRepair Operator Counts(#):")
for name, count in repair_counts.items():
    print(f"{name}: {count}")
print("\nDrone k_opt Counts(#):",drone_k_opt_count)

print("\nExecution time:", execution_time, "seconds")

truck_soc, drone_soc = MultiModalState(current_states[min_index]).soc()[:2]
#truck_time_arrival, drone_time_arrival = MultiModalState(current_states[min_index]).new_time_arrival()
truck_time_arrival, drone_time_arrival = MultiModalState(current_states[min_index]).renew_time_arrival() # 디버깅 완료

total_routes = MultiModalState(current_states[min_index]).routes
truck_current_kwh = data["battery_kwh_t"]
drone_current_kwh = data["battery_kwh_d"]

for i, route in enumerate(total_routes):
    fig, ax1 = plt.subplots(figsize=(8, 6))

    # TRUCK_PATH
    truck_path = [x if route[idx][1] != ONLY_DRONE else None for idx, x in enumerate(truck_soc[i])]
    # 특정 조건을 만족하는 값의 인덱스를 필터링
    excluded_indices_truck = [i for i, value in enumerate(truck_path) if value is None]
    # None 값을 이전 값과 다음 값의 중간 값으로 대체
    for j in range(1, len(truck_path) - 1):
        if truck_path[j] is None:
            left_index = j - 1
            right_index = j + 1
            left_value = None
            right_value = None
            
            # 이전 값이 None이면서 인덱스가 리스트 범위를 벗어나지 않을 때까지 이동
            while left_index >= 0 and truck_path[left_index] is None:
                left_index -= 1
            if left_index >= 0:
                left_value = truck_path[left_index]
            
            # 다음 값이 None이면서 인덱스가 리스트 범위를 벗어나지 않을 때까지 이동
            while right_index < len(truck_path) and truck_path[right_index] is None:
                right_index += 1
            if right_index < len(truck_path):
                right_value = truck_path[right_index]
            
            # 이전 값과 다음 값이 모두 None이 아닌 경우에만 중간 값으로 대체
            if left_value is not None and right_value is not None:
                truck_path[j] = (left_value + right_value) / 2

    # Plot truck data
    ax1.plot(range(len(route)), truck_path, marker='', linestyle='-', label='eTruck', color='blue')
    for iter in range(len(truck_path)):
        if iter in excluded_indices_truck:
            ax1.plot(iter, truck_path[iter], marker='', linestyle='', color='blue')
        else:
            ax1.plot(iter, truck_path[iter], marker='.', color='blue')
    
    # DRONE_PATH
    drone_path = [x if route[idx][1] != ONLY_TRUCK else None for idx, x in enumerate(drone_soc[i])]
    # 특정 조건을 만족하는 값의 인덱스를 필터링
    excluded_indices_drone = [i for i, value in enumerate(drone_path) if value is None]
    # None 값을 이전 값과 다음 값의 중간 값으로 대체
    for j in range(1, len(drone_path) - 1):
        if drone_path[j] is None:
            left_index = j - 1
            right_index = j + 1
            left_value = None
            right_value = None
            
            # 이전 값이 None이면서 인덱스가 리스트 범위를 벗어나지 않을 때까지 이동
            while left_index >= 0 and drone_path[left_index] is None:
                left_index -= 1
            if left_index >= 0:
                left_value = drone_path[left_index]
            
            # 다음 값이 None이면서 인덱스가 리스트 범위를 벗어나지 않을 때까지 이동
            while right_index < len(drone_path) and drone_path[right_index] is None:
                right_index += 1
            if right_index < len(drone_path):
                right_value = drone_path[right_index]
            
            # 이전 값과 다음 값이 모두 None이 아닌 경우에만 중간 값으로 대체
            if left_value is not None and right_value is not None:
                drone_path[j] = (left_value + right_value) / 2
           
    # Plot drone data
    ax1.plot(range(len(route)), drone_path, marker='', linestyle='--', label='eVTOL', color='red')
    for iter in range(len(drone_path)):
        if iter in excluded_indices_drone:
            ax1.plot(iter, drone_path[iter], marker='', linestyle='', color='red')
        else:
            ax1.plot(iter, drone_path[iter], marker='.', color='red')

    ax1.set_xlabel('Customer', fontsize=13, labelpad=10)
    ax1.set_ylabel('State of Charge (%)', fontsize=13, labelpad=10)
    ax1.legend(loc='upper right', fontsize='large')  # 라벨 위치와 크기 조절

    # Show the plot
    plt.title(f"Progress of State of Charge", fontsize=18, pad=20)  # 제목 위로 이동
    plt.grid(True)
    plt.xticks(range(len(route)), [customer[0] for customer in route])
    ax1.tick_params(axis='x', which='major', labelsize=10)  # x축 원소 크기 조절
    ax1.tick_params(axis='y', which='major', labelsize=10)
    plt.subplots_adjust(bottom=0.15, left=0.15)  # x축 레이블과 y축 레이블 이동
    plt.show()

    ## 다음에 꼭 리스트에서 딕셔너리로 바꿔진거 감안해서 ploting 바꾸기 ! 지금은 에러날거임
    # Plot truck and drone elapsed time
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(list(truck_time_arrival[i].keys()), list(truck_time_arrival[i].values()), marker='.', linestyle='-', label='eTruck', color='green')
    ax.plot(list(drone_time_arrival[i].keys()), list(drone_time_arrival[i].values()), marker='.', linestyle='--', label='eVTOL', color='orange')

    # Fill None values
    def fill_none_values(arr):
        filled_arr = arr.copy()
        for i in range(1, len(filled_arr)):
            if filled_arr[i] is None:
                left_index = i - 1
                right_index = i + 1
                while left_index >= 0 and filled_arr[left_index] is None:
                    left_index -= 1
                while right_index < len(filled_arr) and filled_arr[right_index] is None:
                    right_index += 1
                if left_index >= 0 and right_index < len(filled_arr):
                    filled_arr[i] = (filled_arr[left_index] + filled_arr[right_index]) / 2
        return filled_arr

    filled_truck_time = fill_none_values(list(truck_time_arrival[i].values()))
    filled_drone_time = fill_none_values(list(drone_time_arrival[i].values()))

    ax.plot(range(len(route)), filled_truck_time, linestyle='-', color='green',)
    ax.plot(range(len(route)), filled_drone_time, linestyle='--', color='orange')

    ax.set_xlabel('Customer', fontsize=13, labelpad=10)
    ax.set_ylabel('Elapsed Time (min)', fontsize=13, labelpad=10)
    ax.legend(loc='upper left', fontsize='large')  # 라벨 위치와 크기 조절
    plt.title(f"Progress of Elapsed Time", fontsize=18, pad=20)  # 제목 위로 이동
    plt.grid(True)
    plt.xticks(range(len(route)), [customer[0] for customer in route])
    ax.tick_params(axis='x', which='major', labelsize=10)  # x축 원소 크기 조절
    ax.tick_params(axis='y', which='major', labelsize=10)
    plt.subplots_adjust(bottom=0.15, left=0.15)  # x축 레이블과 y축 레이블 이동
    plt.show()