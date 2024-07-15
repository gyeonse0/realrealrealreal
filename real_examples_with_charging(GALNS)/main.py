import copy
import time
import numpy as np
import numpy.random as rnd
import atexit
import pandas as pd
import seaborn as sns
from FileReader import *
from RouteInitializer import *
from RouteGenerator import *
import fileinput
import subprocess

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

SEED = 1234
rnd_state = np.random.RandomState(None)

def initialize_data(vrp_file_path, speed_t, altitude, p_first, drone_battery, drone_payload):
    file_reader = FileReader()
    data = file_reader.read_vrp_file(vrp_file_path)
    
    # 변수 업데이트
    data["speed_t"] = speed_t
    data["altitude"] = altitude
    data["drone_battery"] = drone_battery
    data["drone_payload"] = drone_payload
    
    time_matrix = np.array(data["edge_km_t"]) / speed_t
    data["priority_delivery_time"] = file_reader.randomize_vrp_data(data["priority_delivery_time"], time=time_matrix, key_seed=56, p_first=p_first)
    data["logistic_load"] = file_reader.randomize_vrp_data(data["logistic_load"], max_value=8, key_seed=78)
    data["availability_landing_spot"] = file_reader.randomize_vrp_data(data["availability_landing_spot"], key_seed=12)
    data["customer_drone_preference"] = file_reader.randomize_vrp_data(data["customer_drone_preference"], key_seed=34)
   

    # price는 무게에 의존
    logistic_load = np.array(list(data['logistic_load'].values()))
    price = np.zeros_like(logistic_load)
    price[logistic_load > 7] = 10
    price[(logistic_load <= 7) & (logistic_load > 5)] = 7
    price[(logistic_load <= 5) & (logistic_load > 3)] = 5
    price[logistic_load <= 3] = 3
    data["price"] = {key: value for key, value in zip(data["price"].keys(), price)}

    # 업데이트된 데이터 파일 쓰기
    file_reader.update_vrp_file(vrp_file_path, data)
    
    return data

from SolutionPlotter import *
from MultiModalState import *
from Destroy import *
from Repair import *

def run_simulation(data):
    destroyer = Destroy()
    Rep = Repair()
    initializer = RouteInitializer(data)
    initial_truck = initializer.init_truck()

    destroy_operators = [destroyer.random_removal, destroyer.can_drone_removal, destroyer.high_cost_removal, destroyer.priority_customer_removal]

    repair_operators = [Rep.random_repair, Rep.drone_first_truck_second, Rep.truck_first_drone_second, Rep.heavy_truck_repair, Rep.light_drone_repair]
                        #Rep.regret_random_repair, Rep.regret_drone_first_truck_second, Rep.regret_truck_first_drone_second, Rep.regret_heavy_truck_repair, Rep.regret_light_drone_repair]
                        
    destroy_selector = RouletteWheel()
    repair_selector = RouletteWheel()

    destroy_selector.set_operators(destroy_operators)
    repair_selector.set_operators(repair_operators)

    destroy_counts = {destroyer.__name__: 0 for destroyer in destroy_operators}
    repair_counts = {repairer.__name__: 0 for repairer in repair_operators}
    ga_count=0

    # 초기 설정
    iteration_num=10

    start_temperature = 100
    end_temperature = 0.01
    step = 0.1

    temperature = start_temperature
    current_num=0
    outcome_counts = {1: 0, 2: 0, 3: 0, 4: 0}
    start_time = time.time()
    min_objective_time = None

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
            

        # 이때 새로 갱신했으면 이때 min_objective_time을 기억하도록 하기    
        #elif current_num % 100 == 0 and current_num != 0:
            
            #genetic_state = Rep.genetic_algorithm(current_states[-1],population_size=5, generations=100, mutation_rate=0.2)

            #current_states.append(genetic_state)
            #objective_value = MultiModalState(genetic_state).cost_objective()
            #objectives.append(objective_value)
            #if objective_value == min(objectives):
                #min_objective_time = time.time()
            #temperature = max(end_temperature, temperature - step)
            #current_num+=1
            #ga_count+=1
        
        #elif current_num % 10 == 0 and current_num != 0:
            
            #k_opt_state = Rep.drone_k_opt(current_states[-1],rnd_state)

            #current_states.append(k_opt_state)
            #objective_value = MultiModalState(k_opt_state).cost_objective()
            #objectives.append(objective_value)
            
            #temperature = max(end_temperature, temperature - step)
            #current_num+=1
            #ga_count+=1

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
                    min_objective_time = time.time()

                    genetic_state = Rep.genetic_algorithm(repaired_state, population_size=5, generations=100, mutation_rate=0.2)

                    current_states.append(genetic_state)
                    objective_value = MultiModalState(genetic_state).cost_objective()
                    objectives.append(objective_value)
                    if objective_value == min(objectives):
                        min_objective_time = time.time()
                    temperature = max(end_temperature, temperature - step)
                    ga_count+=1

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
    min_objective_time = min_objective_time - start_time

    return min_objective, current_states[min_index], min_objective_time, execution_time

def print_current_parameters(data, extra_params, ofv):
    parameters = {
        'SPEED_T': data.get('speed_t', 'N/A'),
        'ALTITUDE': data.get('altitude', 'N/A'),
        'p_first': extra_params.get('p_first', 'N/A'),
        'drone_battery': extra_params.get('drone_battery', 'N/A'),
        'drone_payload': extra_params.get('drone_payload', 'N/A')
    }
    ofv_formatted = f'{ofv:.2f} USD'
    print(', '.join(f'{key}: {value}' for key, value in parameters.items()) + f', OFV: {ofv_formatted}')

def save_to_excel(filename, sheetname, results):
    df = pd.DataFrame(results)
    with pd.ExcelWriter(filename, mode='a', engine='openpyxl') as writer:  # mode='a'는 파일에 추가하는 모드입니다.
        df.to_excel(writer, sheet_name=sheetname, index=False)


# 베이스라인 값 설정
baseline_speed_t = 1.0

baseline_head = 50
baseline_altitude = 20
baseline_p_first = 0.6
baseline_drone_battery = 1.0
baseline_drone_payload = 30

# 테스트할 SPEED_T 값 리스트
speed_t_values = [0.5, 0.8, 1.0, 1.2]
speed_t_objective_values = []

# 테스트할 ALTITUDE 값 리스트
altitude_values = [20, 30, 40, 50]
altitude_objective_values = []

# 테스트할 p_first 값 리스트
p_first_values = [0.5, 0.6, 0.7, 0.8]
p_first_objective_values = []

# 테스트할 drone_battery 값 리스트
drone_battery_values = [0.8, 1.0, 1.2, 1.4]
drone_battery_objective_values = []

# 테스트할 drone_payload 값 리스트
drone_payload_values = [10, 30, 50, 70]
drone_payload_objective_values = []

vrp_file_path = r'C:\Users\User\OneDrive\바탕 화면\realrealrealreal-main\realrealrealreal-main\real_examples_with_charging(GALNS)\data\multi_modal_data.vrp'
backup_file_path = r'C:\Users\User\OneDrive\바탕 화면\realrealrealreal-main\realrealrealreal-main\real_examples_with_charging(GALNS)\data\multi_modal_data_backup.vrp'

plotter = SolutionPlotter({})

# 파일 백업
file_reader = FileReader()
file_reader.create_backup(vrp_file_path, backup_file_path)

# 민감도 분석 결과 저장용 리스트 초기화
results = []

# 베이스라인 결과 한 번만 출력
data = initialize_data(vrp_file_path, baseline_speed_t, baseline_altitude, baseline_p_first, baseline_drone_battery, baseline_drone_payload)
revenue=0
for i in range(1, data["dimension"]):
    revenue = revenue + data["price"][i]
best_objective, best_solution, min_objective_time, execution_time = run_simulation(data)
best_solution.cost_objective()
results.append({'Population(#)': baseline_head,'ALTITUDE(m)': baseline_altitude, 'Fast Demand(%)': baseline_p_first * 100, 'Drone Battery(kWh)':baseline_drone_battery, 'Drone Payload(kg)':baseline_drone_payload, 'Carrier Cost': best_solution.get_carrier_cost(), 'Energy Cost': best_solution.get_energy_cost(), 'Refund Cost': best_solution.get_refund_cost(), 'Total Cost': best_objective, 'Revenue': revenue, 'Net Profit': revenue-best_objective, 'Min Objective Time': min_objective_time, 'Execution_time': execution_time})
print_current_parameters(data, {'p_first': baseline_p_first, 'drone_battery': baseline_drone_battery, 'drone_payload': baseline_drone_payload}, best_objective)
plotter.data = data  # plotter의 데이터를 현재 데이터로 업데이트
#plotter.plot_current_solution(best_solution)


# 민감도 분석을 위한 반복 실행
# for speed_t in speed_t_values:
#     if speed_t == baseline_speed_t:
#         continue
#     data = initialize_data(vrp_file_path, speed_t, baseline_altitude, baseline_p_first, baseline_drone_battery, baseline_drone_payload)
#     best_objective, best_solution = run_simulation(data)
#     speed_t_objective_values.append(best_objective)
#     results.append({'Parameter': 'SPEED_T', 'Value': speed_t, 'Objective': best_objective})
#     print_current_parameters(data, {'p_first': baseline_p_first, 'drone_battery': baseline_drone_battery, 'drone_payload': baseline_drone_payload}, best_objective)
#     plotter.data = data  # plotter의 데이터를 현재 데이터로 업데이트
#     plotter.plot_current_solution(best_solution)


for altitude in altitude_values:
    if altitude == baseline_altitude:
        continue
    data = initialize_data(vrp_file_path, baseline_speed_t, altitude, baseline_p_first, baseline_drone_battery, baseline_drone_payload)
    best_objective, best_solution, min_objective_time, execution_time = run_simulation(data)
    revenue=0
    for i in range(1, data["dimension"]):
        revenue = revenue + data["price"][i]
    altitude_objective_values.append(best_objective)
    best_solution.cost_objective()
    results.append({'Population(#)': baseline_head, 'ALTITUDE(m)': altitude, 'Fast Demand(%)': baseline_p_first * 100, 'Drone Battery(kWh)':baseline_drone_battery, 'Drone Payload(kg)':baseline_drone_payload, 'Carrier Cost': best_solution.get_carrier_cost(), 'Energy Cost': best_solution.get_energy_cost(), 'Refund Cost': best_solution.get_refund_cost(), 'Total Cost': best_objective, 'Revenue': revenue, 'Net Profit': revenue-best_objective, 'Min Objective Time': min_objective_time, 'Execution_time': execution_time})
    print_current_parameters(data, {'p_first': baseline_p_first, 'drone_battery': baseline_drone_battery, 'drone_payload': baseline_drone_payload}, best_objective)
    plotter.data = data  # plotter의 데이터를 현재 데이터로 업데이트
    #plotter.plot_current_solution(best_solution)

for p_first in p_first_values:
    if p_first == baseline_p_first:
        continue
    data = initialize_data(vrp_file_path, baseline_speed_t, baseline_altitude, p_first, baseline_drone_battery, baseline_drone_payload)
    best_objective, best_solution, min_objective_time, execution_time = run_simulation(data)
    revenue=0
    for i in range(1, data["dimension"]):
        revenue = revenue + data["price"][i]
    p_first_objective_values.append(best_objective)
    best_solution.cost_objective()
    results.append({'Population(#)': baseline_head, 'ALTITUDE(m)': baseline_altitude, 'Fast Demand(%)': p_first * 100, 'Drone Battery(kWh)':baseline_drone_battery, 'Drone Payload(kg)':baseline_drone_payload, 'Carrier Cost': best_solution.get_carrier_cost(), 'Energy Cost': best_solution.get_energy_cost(), 'Refund Cost': best_solution.get_refund_cost(), 'Total Cost': best_objective, 'Revenue': revenue, 'Net Profit': revenue-best_objective, 'Min Objective Time': min_objective_time, 'Execution_time': execution_time})
    print_current_parameters(data, {'p_first': p_first, 'drone_battery': baseline_drone_battery, 'drone_payload': baseline_drone_payload}, best_objective)
    plotter.data = data  # plotter의 데이터를 현재 데이터로 업데이트
    #plotter.plot_current_solution(best_solution)

# drone_battery 민감도 분석
for drone_battery in drone_battery_values:
    if drone_battery == baseline_drone_battery:
        continue
    data = initialize_data(vrp_file_path, baseline_speed_t, baseline_altitude, baseline_p_first, drone_battery, baseline_drone_payload)
    best_objective, best_solution, min_objective_time, execution_time = run_simulation(data)
    revenue=0
    for i in range(1, data["dimension"]):
        revenue = revenue + data["price"][i]
    drone_battery_objective_values.append(best_objective)
    best_solution.cost_objective()
    results.append({'Population(#)': baseline_head, 'ALTITUDE(m)': baseline_altitude, 'Fast Demand(%)': baseline_p_first * 100, 'Drone Battery(kWh)':drone_battery, 'Drone Payload(kg)':baseline_drone_payload, 'Carrier Cost': best_solution.get_carrier_cost(), 'Energy Cost': best_solution.get_energy_cost(), 'Refund Cost': best_solution.get_refund_cost(), 'Total Cost': best_objective, 'Revenue': revenue, 'Net Profit': revenue-best_objective, 'Min Objective Time': min_objective_time, 'Execution_time': execution_time})
    print_current_parameters(data, {'p_first': baseline_p_first, 'drone_battery': drone_battery, 'drone_payload': baseline_drone_payload}, best_objective)
    plotter.data = data  # plotter의 데이터를 현재 데이터로 업데이트
    #plotter.plot_current_solution(best_solution)

# drone_payload 민감도 분석
for drone_payload in drone_payload_values:
    if drone_payload == baseline_drone_payload:
        continue
    data = initialize_data(vrp_file_path, baseline_speed_t, baseline_altitude, baseline_p_first, baseline_drone_battery, drone_payload)
    best_objective, best_solution, min_objective_time, execution_time = run_simulation(data)
    revenue=0
    for i in range(1, data["dimension"]):
        revenue = revenue + data["price"][i]
    drone_payload_objective_values.append(best_objective)
    best_solution.cost_objective()
    results.append({'Population(#)': baseline_head, 'ALTITUDE(m)': baseline_altitude, 'Fast Demand(%)': baseline_p_first * 100, 'Drone Battery(kWh)':baseline_drone_battery, 'Drone Payload(kg)':drone_payload, 'Carrier Cost': best_solution.get_carrier_cost(), 'Energy Cost': best_solution.get_energy_cost(), 'Refund Cost': best_solution.get_refund_cost(), 'Total Cost': best_objective, 'Revenue': revenue, 'Net Profit': revenue-best_objective, 'Min Objective Time': min_objective_time, 'Execution_time': execution_time})
    print_current_parameters(data, {'p_first': baseline_p_first, 'drone_battery': baseline_drone_battery, 'drone_payload': drone_payload}, best_objective)
    plotter.data = data  # plotter의 데이터를 현재 데이터로 업데이트
    #plotter.plot_current_solution(best_solution)

# 결과 엑셀 파일에 저장
save_to_excel(r'C:\Users\User\OneDrive\바탕 화면\sensitivity_analysis_results.xlsx', 'Resulthhgglkjhlkjhs', results)

# ALTITUDE 결과를 플롯팅
plt.figure(figsize=(10, 6))
altitude_full_values = [baseline_altitude] + [v for v in altitude_values if v != baseline_altitude]
altitude_full_objectives = [best_objective] + altitude_objective_values
sns.barplot(x=altitude_full_values, y=altitude_full_objectives, hue=altitude_full_values, palette="Greens_d", dodge=False, legend=False)
plt.title('Objective Function Value vs ALTITUDE')
plt.xlabel('ALTITUDE')
plt.ylabel('Objective Function Value (USD)')
plt.grid(True)
plt.show()

# p_first 결과를 플롯팅
plt.figure(figsize=(10, 6))
p_first_full_values = [baseline_p_first] + [v for v in p_first_values if v != baseline_p_first]
p_first_full_objectives = [best_objective] + p_first_objective_values
sns.barplot(x=p_first_full_values, y=p_first_full_objectives, hue=p_first_full_values, palette="Oranges_d", dodge=False, legend=False)
plt.title('Objective Function Value vs p_first')
plt.xlabel('p_first')
plt.ylabel('Objective Function Value (USD)')
plt.grid(True)
plt.show()

# drone_battery 결과를 플롯팅
plt.figure(figsize=(10, 6))
drone_battery_full_values = [baseline_drone_battery] + [v for v in drone_battery_values if v != baseline_drone_battery]
drone_battery_full_objectives = [best_objective] + drone_battery_objective_values
sns.barplot(x=drone_battery_full_values, y=drone_battery_full_objectives, hue=drone_battery_full_values, palette="Purples_d", dodge=False, legend=False)
plt.title('Objective Function Value vs Drone Battery')
plt.xlabel('Drone Battery')
plt.ylabel('Objective Function Value (USD)')
plt.grid(True)
plt.show()

# drone_payload 결과를 플롯팅
plt.figure(figsize=(10, 6))
drone_payload_full_values = [baseline_drone_payload] + [v for v in drone_payload_values if v != baseline_drone_payload]
drone_payload_full_objectives = [best_objective] + drone_payload_objective_values
sns.barplot(x=drone_payload_full_values, y=drone_payload_full_objectives, hue=drone_payload_full_values, palette="Reds_d", dodge=False, legend=False)
plt.title('Objective Function Value vs Drone Payload')
plt.xlabel('Drone Payload')
plt.ylabel('Objective Function Value (USD)')
plt.grid(True)
plt.show()


atexit.register(file_reader.restore_on_exit, backup_file_path, vrp_file_path)
