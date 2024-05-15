import copy
import random
from types import SimpleNamespace
import vrplib 
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import numpy.random as rnd
from typing import List
from itertools import groupby

from RouteGenerator import *
from FileReader import *


vrp_file_path = r'C:\Users\User\OneDrive\바탕 화면\examples_with_charging1\data\multi_modal_data.vrp'

file_reader = FileReader()
data = file_reader.read_vrp_file(vrp_file_path)

IDLE = 0 # 해당 노드에 드론이 트럭에 업힌 상태의 경우
FLY = 1 # 해당 노드에서 트럭이 드론의 임무를 위해 드론을 날려주는 경우
ONLY_DRONE = 2 # 해당 노드에 드론만이 임무를 수행하는 서비스 노드인 경우
CATCH = 3 # 해당 노드에서 트럭이 임무를 마친 드론을 받는 경우
ONLY_TRUCK = 4 # 해당 노드에서 트럭만이 임무를 수행하는 경우 (드론이 업혀있지 않음)
NULL = None # 해당 노드가 전체 ROUTES에 할당이 안되어 있는 경우 (SOLUTION에서 UNVISITED 한 노드)

class MultiModalState:
    """
    routes 딕셔너리 집합을 input으로 받아서 copy를 수행한 뒤, 해당 routes 에서의 정보를 추출하는 함수
    output: objective cost value / 특정 customer node를 포함한 route  
    """
    def __init__(self, routes, unassigned=None):
        self.routes = routes
        self.unassigned = unassigned if unassigned is not None else []
        
        unassigned_check=[]
        for node_id in range(1, data["dimension"]):
            is_in_routes = any(node_id == node[0] for route in routes for node in route)
            if not is_in_routes and (node_id, None) not in unassigned_check:
                unassigned_check.append((node_id, None))
        
        self.unassigned = unassigned_check

    def copy(self):
        return MultiModalState(
            copy.deepcopy(self.routes),
            self.unassigned.copy()
        )
        
    def __str__(self):
        return f"Routes: {self.routes}, Unassigned: {self.unassigned}" 
    
    def __iter__(self):
        return iter(self.routes)       
        
        
    def charging_objective(self):
        charging_objective = 0
        for route in self.routes:
            truck_path = [value for value in route if value[1] != ONLY_DRONE]
            start_index = 0
            truck_drive_only = True
            truck_drive_with_drone = False
            for idx, customer in enumerate(truck_path):
                last_condition = (idx == len(truck_path) - 1) and truck_drive_with_drone
                if (customer[1] == CATCH and truck_drive_only) or last_condition:
                    truck_drive_only = False
                    truck_drive_with_drone = True
                    start_index = idx
                elif customer[1] == FLY and truck_drive_with_drone: 
                    ### 이 인덱스까지 모두 더해준다
                    end_index = idx
                    truck_idle_distance = 0
                    for k in range(start_index, end_index):
                        truck_idle_distance += data["edge_km_t"][truck_path[k][0]][truck_path[k+1][0]]
                    truck_idle_time = truck_idle_distance / data["speed_t"]

                    charging_objective += (data["charging_kw_d"]) * (truck_idle_time/60)

                    truck_drive_with_drone = False
                    truck_drive_only = True
                    start_index = idx
        return charging_objective

        
    
    def objective(self):
        """
        data와 routes 딕셔너리 집합을 이용하여 objective value 계산해주는 함수
        our objective cost value = energy_consunmption(kwh)
        energy_consunmption(kwh)={Truck edge cost(km), Truck energy consumption(kwh/km), Drone edge cost(km), Drone energy consumption(kwh/km)}
        TO DO: 이후에 logistic_load 등의 데이터 등을 추가로 활용하여 energy_consumption 모델링 확장 필요
        """
        
        #여기 에너지소모에 지금 트럭의 에너지 소모 (드론에게 충전해주는 거 빠짐)
        
        divided_routes = apply_dividing_route_to_routes(self.routes)
        energy_consumption = 0.0

        for route_info in divided_routes:
            vtype = route_info['vtype']
            path = route_info['path']

            if vtype == 'truck':
                for i in range(len(path) - 1):
                    loc_from = path[i][0] if isinstance(path[i], tuple) else path[i]
                    loc_to = path[i+1][0] if isinstance(path[i+1], tuple) else path[i+1]

                    edge_weight = data["edge_km_t"][loc_from][loc_to]
                    energy_consumption += edge_weight * data["energy_kwh/km_t"]

            
            elif vtype == 'drone': #드론은 1(fly)부터 3(catch)까지만의 edge를 반복적으로 고려해준다는 알고리즘
                flag = 0
                for j in range(len(path)):
                    if flag == 0 and path[j][1] == 1:
                        start_index = j
                        flag = 1
                    elif path[j][1] == 3 and flag == 1:
                        for k in range(start_index, j):
                            edge_weight = data["edge_km_d"][path[k][0]][path[k+1][0]]
                            energy_consumption += edge_weight * data["energy_kwh/km_d"]
                        flag=0
                        
        return energy_consumption 
    
    def truck_consumtion(self):
        divided_routes = apply_dividing_route_to_routes(self.routes)
        energy_consumption = 0.0

        for route_info in divided_routes:
            vtype = route_info['vtype']
            path = route_info['path']


            if vtype == 'truck':
                for i in range(len(path) - 1):
                    loc_from = path[i][0] if isinstance(path[i], tuple) else path[i]
                    loc_to = path[i+1][0] if isinstance(path[i+1], tuple) else path[i+1]

                    edge_weight = data["edge_km_t"][loc_from][loc_to]
                    energy_consumption += edge_weight * data["energy_kwh/km_t"]

        return energy_consumption
    
    def drone_consumption(self):
        divided_routes = apply_dividing_route_to_routes(self.routes)
        energy_consumption = 0.0

        for route_info in divided_routes:
            vtype = route_info['vtype']
            path = route_info['path']

            if vtype == 'drone': #드론은 1(fly)부터 3(catch)까지만의 edge를 반복적으로 고려해준다는 알고리즘
                flag = 0
                current_logistic_load=0
                for j in range(len(path)):
                    if flag == 0 and path[j][1] == 1:
                        start_index = j
                        flag = 1
                    elif (path[j][1] == 3 or path[j][1] == 1) and flag == 1:
                        for k in range(start_index, j):
                            if path[k][1] == 2:
                                current_logistic_load += data["logistic_load"][path[k][0]]        

                        for k in range(start_index, j):
                            edge_weight = data["edge_km_d"][path[k][0]][path[k+1][0]]
                            edge_time = (edge_weight / (data["speed_d"]))/60 #hour
                            if path[k+1][1]==2:
                                energy_consumption += self.detail_drone_modeling(current_logistic_load,edge_time)
                                current_logistic_load -= data["logistic_load"][path[k+1][0]] 
                            elif path[k+1][1]==1 or path[k+1]==3:
                                energy_consumption += self.detail_just_drone_modeling(edge_time)
                                current_logistic_load = 0
                        flag=0
                        
        return energy_consumption
    
    def cost_objective(self): #지금 디테일 모델링 사용중이지 않음. 주행거리 효율로만 사용
        total_energy_consumption = self.soc()[2]

        energy_cost = total_energy_consumption * 0.316

        refund = 0
        ## 아직은 잘 모르겠는데 자료구조 적으로 priority time이 0이 아닌 것에만 접근하고 그것의 차이만 더할 수 있으면 좋을 것 같음
        for route in self.routes:
            late_time = 0
            truck_time = self.new_time_arrival_per_route(route)['eTruck']
            drone_time = self.new_time_arrival_per_route(route)['eVTOL']
            for customer in route[1:]: # time arrival이 데이터보다 더 클 때
                if data["priority_delivery_time"][customer[0]] != 0:
                    if truck_time[customer[0]] is not None:
                        late_time += (truck_time[customer[0]] - data["priority_delivery_time"][customer[0]])
                    else:
                        late_time += (drone_time[customer[0]] - data["priority_delivery_time"][customer[0]])
                    
                    if late_time > 0:
                        refund += np.exp(late_time / 10)

            # priority_customers = filter(lambda customer: data["priority_delivery_time"][customer[0]] != 0 and customer[0] != 'source', truck_time.items())
            # next(priority_customers)
            # for priority_customer, time in priority_customers:
            #     if time is not None:
            #         late_time += (time - data["priority_delivery_time"][priority_customer])
            #     else:
            #         late_time += (drone_time[priority_customer] - data["priority_delivery_time"][priority_customer])
            #     if late_time > 0:
            #         refund = np.exp(late_time)
        

        time_of_eTrucks = self.renew_time_arrival()[0]
        carrier_cost = 0
        salary = 15
        for carrier in time_of_eTrucks:
            if carrier:
                carrier_cost += (carrier[0]/60) * salary  # sink에 도착했을 때 carrier의 time arrival
            else:
                print("경고: 빈 리스트가 포함되어 있습니다.")    

        return energy_cost + carrier_cost + refund

    
    def new_objective(self):

        divided_routes = apply_dividing_route_to_routes(self.routes)
        energy_consumption = 0.0

        for route_info in divided_routes:
            vtype = route_info['vtype']
            path = route_info['path']

            if vtype == 'truck':
                for i in range(len(path) - 1):
                    loc_from = path[i][0] if isinstance(path[i], tuple) else path[i]
                    loc_to = path[i+1][0] if isinstance(path[i+1], tuple) else path[i+1]

                    edge_weight = data["edge_km_t"][loc_from][loc_to]
                    energy_consumption += edge_weight * data["energy_kwh/km_t"]

            
            elif vtype == 'drone': #드론은 1(fly)부터 3(catch)까지만의 edge를 반복적으로 고려해준다는 알고리즘
                flag = 0
                current_logistic_load=0
                for j in range(len(path)):
                    if flag == 0 and path[j][1] == 1:
                        start_index = j
                        flag = 1
                    elif (path[j][1] == 3 or path[j][1] == 1) and flag == 1:
                        for k in range(start_index, j):
                            if path[k][1] == 2:
                                current_logistic_load += data["logistic_load"][path[k][0]]        

                        for k in range(start_index, j):
                            edge_weight = data["edge_km_d"][path[k][0]][path[k+1][0]]
                            edge_time = (edge_weight / (data["speed_d"]))/60 #hour
                            if path[k+1][1]==2:
                                energy_consumption += self.detail_drone_modeling(current_logistic_load,edge_time)
                                current_logistic_load -= data["logistic_load"][path[k+1][0]] 
                            elif path[k+1][1]==1 or path[k+1]==3:
                                energy_consumption += self.detail_just_drone_modeling(edge_time)
                                current_logistic_load = 0
                        flag=0
                        
        return energy_consumption
    
    ##지금 이거 충전고려 안해줄때 해주려고 + self.charging_objective() 빼준거임 
    
    def detail_drone_modeling(self,current_logistic_load,edge_time):
        
        drone_consumption = ((((data["mass_d"]+current_logistic_load)*data["speed_d"]*60)/(370*data["lift_to_drag"]*data["power_motor_prop"]))+data["power_elec"])*edge_time
        return drone_consumption
    
    def detail_just_drone_modeling(self,edge_time):
        
        drone_consumption = ((((data["mass_d"])*data["speed_d"]*60)/(370*data["lift_to_drag"]*data["power_motor_prop"]))+data["power_elec"])*edge_time
        return drone_consumption
    
    def truck_objective(truck_path, data):
        energy_consumption = 0

        for i in range(len(truck_path) - 1):
            loc_from = truck_path[i][0] if isinstance(truck_path[i], tuple) else truck_path[i]
            loc_to = truck_path[i + 1][0] if isinstance(truck_path[i + 1], tuple) else truck_path[i + 1]

            edge_weight = data["edge_km_t"][loc_from][loc_to]
            energy_consumption += edge_weight * data["energy_kwh/km_t"]

        return energy_consumption
    
    def max_time(self): #각 route의 (트럭바퀴굴러간 시간 + 프로펠러돌아간 시간) 중 최대값
        time = []
        for route in self.routes:
            truck_time = 0
            drone_time = 0
            time_sum = [0]
            jump = 0
            flee = 0
            for idx in range(1, len(route)):
                if route[idx][1] == CATCH or ((route[idx][1] == FLY) and (route[idx-1][1] in [ONLY_TRUCK, ONLY_DRONE])):
                    drone_time += data["edge_km_d"][route[idx-flee-1][0]][route[idx][0]]/data["speed_d"]
                    truck_time += data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]]/data["speed_t"]
                    flee = 0
                    jump = 0
                    time_sum.append(truck_time + drone_time)
                elif route[idx][1] == ONLY_DRONE:
                    drone_time += data["edge_km_d"][route[idx-flee-1][0]][route[idx][0]]/data["speed_d"]
                    flee = 0
                    time_sum.append(drone_time)
                    jump += 1
                elif route[idx][1] == ONLY_TRUCK:
                    truck_time += data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]]/data["speed_t"]
                    jump = 0
                    time_sum.append(truck_time)
                    flee += 1
                elif route[idx][1] in [IDLE, FLY]:
                    truck_time += data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]]/data["speed_t"]
                    time_sum.append(truck_time)
            time.append(time_sum)
        
        objective_time = 0
        for i in range(len(time)):
            if time[i][-1] > objective_time:
                objective_time = time[i][-1]

        return objective_time
    
    def all_time(self):# 각 route의 (트럭바퀴굴러간 시간 + 프로펠러돌아간 시간) 을 모두 합한 값
        time = []
        for route in self.routes:
            truck_time = 0
            drone_time = 0
            time_sum = [0]
            jump = 0
            flee = 0
            for idx in range(1, len(route)):
                if route[idx][1] == CATCH or ((route[idx][1] == FLY) and (route[idx-1][1] in [ONLY_TRUCK, ONLY_DRONE])):
                    drone_time += data["edge_km_d"][route[idx-flee-1][0]][route[idx][0]]/data["speed_d"]
                    truck_time += data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]]/data["speed_t"]
                    flee = 0
                    jump = 0
                    time_sum.append(truck_time + drone_time)
                elif route[idx][1] == ONLY_DRONE:
                    drone_time += data["edge_km_d"][route[idx-flee-1][0]][route[idx][0]]/data["speed_d"]
                    flee = 0
                    time_sum.append(drone_time)
                    jump += 1
                elif route[idx][1] == ONLY_TRUCK:
                    truck_time += data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]]/data["speed_t"]
                    jump = 0
                    time_sum.append(truck_time)
                    flee += 1
                elif route[idx][1] in [IDLE, FLY]:
                    truck_time += data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]]/data["speed_t"]
                    time_sum.append(truck_time)
            time.append(time_sum)

        objective_time = 0
        for i in range(len(time)):
            objective_time += time[i][-1]

        return objective_time
    
    def new_all_time(self):
        divided_routes = apply_dividing_route_to_routes(self.routes)
        truck_time = 0
        drone_time = 0

        for route_info in divided_routes:
            vtype = route_info['vtype']
            path = route_info['path']

            if vtype == 'truck':
                for i in range(len(path) - 1):
                    loc_from = path[i][0] if isinstance(path[i], tuple) else path[i]
                    loc_to = path[i+1][0] if isinstance(path[i+1], tuple) else path[i+1]

                    truck_time += (data["edge_km_t"][loc_from][loc_to])/data["speed_t"]

            elif vtype == 'drone': #드론은 1(fly)부터 3(catch)까지만의 edge를 반복적으로 고려해준다는 알고리즘
                flag = 0
                for j in range(len(path)):
                    if flag == 0 and path[j][1] == 1:
                        start_index = j
                        flag = 1
                    elif path[j][1] == 3 and flag == 1:
                        for k in range(start_index, j):
                            drone_time += (data["edge_km_d"][path[k][0]][path[k+1][0]])/data["speed_d"]
                        flag=0

        all_time= truck_time + drone_time
                        
        return all_time 
    

    def new_max_time(self):
        divided_routes = apply_dividing_route_to_routes(self.routes)
        truck_time = 0
        drone_time = 0
        time = []
        for route_info in divided_routes:
            vtype = route_info['vtype']
            path = route_info['path']

            if vtype == 'truck':
                for i in range(len(path) - 1):
                    loc_from = path[i][0] if isinstance(path[i], tuple) else path[i]
                    loc_to = path[i+1][0] if isinstance(path[i+1], tuple) else path[i+1]

                    truck_time += (data["edge_km_t"][loc_from][loc_to])/data["speed_t"]

            elif vtype == 'drone': #드론은 1(fly)부터 3(catch)까지만의 edge를 반복적으로 고려해준다는 알고리즘
                flag = 0
                for j in range(len(path)):
                    if flag == 0 and path[j][1] == 1:
                        start_index = j
                        flag = 1
                    elif path[j][1] == 3 and flag == 1:
                        for k in range(start_index, j):
                            drone_time += (data["edge_km_d"][path[k][0]][path[k+1][0]])/data["speed_d"]
                        flag=0
                        
            each_time= truck_time + drone_time
            time.append(each_time)
        
        max_time=0
        for i in range(time):
            if max_time< time[i]:
                max_time =time[i]
                        
        return max_time
    
    def drone_soc(self):
        routes_d = []
        ofvs = []
        charging_kwh=0
        for route in self.routes:
            #drone_path = [value[0] for value in route if value[1] != ONLY_TRUCK]
            start_index = 0
            first = True
            truck_drive_only = True
            truck_drive_with_drone = False
            drone_current_kwh = data["battery_kwh_d"]
            drone_distance = 0
            drone_idle_distance = 0
            ofv = []
            customer_by_drone = [customer[0] for customer in route]
            for idx, customer in enumerate(route):
                # 맨처음 반영해주기 위해 추가
                last_condition = (idx == len(route) - 1) and truck_drive_with_drone
                if customer[1] == FLY and first:
                    start_index = idx
                    ofv = [drone_current_kwh] * (idx+1)
                    first = False
                # 얘까지는 ofv 그대로 채우기
                elif (customer[1] == CATCH and truck_drive_only):
                    # <드론이 미션 수행하는 케이스>, 이 인덱스까지 모두 더해준다
                    end_index = idx
                    for k in range(start_index, end_index):
                        if route[k][1] == ONLY_TRUCK:
                            ofv.append(drone_current_kwh)
                            continue
                        drone_distance = data["edge_km_d"][route[k][0]][route[k+1][0]]
                        energy_consumption = drone_distance * data["energy_kwh/km_d"]
                        drone_current_kwh -= energy_consumption
                        ofv.append(drone_current_kwh)

                    truck_drive_only = False
                    truck_drive_with_drone = True
                    start_index = idx
                elif customer[1] == FLY and truck_drive_with_drone: 
                    # <드론이 업혀있는 케이스>, 이 인덱스까지 모두 더해준다
                    end_index = idx
                    for k in range(start_index, end_index):
                        drone_idle_distance = data["edge_km_t"][route[k][0]][route[k+1][0]]
                        #energy_consumption = drone_idle_distance * data["energy_kwh/km_t"]
                        drone_idle_time = drone_idle_distance / data["speed_d"]
                        #drone_current_kwh -= energy_consumption
                        if drone_current_kwh < data["battery_kwh_d"]: # 만땅이 아니라면
                            drone_current_kwh += (data["charging_kw_d"]) * (drone_idle_time/60) #kwh 단위 고려
                            charging_kwh += (data["charging_kw_d"]) * (drone_idle_time/60)
                            if drone_current_kwh > data["battery_kwh_d"]:
                                drone_current_kwh = data["battery_kwh_d"] #충전해도 최대 배터리 양은 넘을 수 없음
                                charging_kwh += 0
                        ofv.append(drone_current_kwh)
                    truck_drive_with_drone = False
                    truck_drive_only = True
                    start_index = idx
                elif last_condition:
                    ofv.extend([drone_current_kwh] * (idx - end_index))
                # 정말 한 라우트의 모든 비짓 타입이 꽁일 경우도 생각해줘야할듯 
                elif(idx == len(route) - 1) and truck_drive_only:
                    ofv = [drone_current_kwh] * (idx+1)
            soc = [x / data["battery_kwh_d"] * 100 for x in ofv]
            ofvs.append(soc)
            routes_d.append(customer_by_drone)
        return routes_d, ofvs
    
    def soc(self):
        truck_soc = []
        drone_soc = []
        for route in self.routes:
            truck_energy_consumption = 0
            drone_energy_consumption = 0
            total_energy_consumption = 0
            truck_kwh = data["battery_kwh_t"]
            drone_kwh = data["battery_kwh_d"]
            truck_ofvs = [truck_kwh]
            drone_ofvs = [drone_kwh]
            jump = 0
            flee = 0
            for idx in range(1, len(route)):
                if route[idx][1] == CATCH or ((route[idx][1] == FLY) and (route[idx-1][1] in [ONLY_TRUCK, ONLY_DRONE])):
                    truck_energy_consumption = data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]] * data["energy_kwh/km_t"]
                    drone_energy_consumption = data["edge_km_d"][route[idx-flee-1][0]][route[idx][0]] * data["energy_kwh/km_d"]
                    total_energy_consumption += (truck_energy_consumption + drone_energy_consumption)
                    flee = 0
                    jump = 0
                    truck_kwh -= truck_energy_consumption
                    drone_kwh -= drone_energy_consumption
                    truck_ofvs.append(truck_kwh)
                    drone_ofvs.append(drone_kwh)
                elif route[idx][1] == ONLY_DRONE:
                    drone_energy_consumption = data["edge_km_d"][route[idx-flee-1][0]][route[idx][0]] * data["energy_kwh/km_d"]
                    total_energy_consumption += drone_energy_consumption
                    flee = 0
                    drone_kwh -= drone_energy_consumption
                    truck_ofvs.append(truck_kwh)
                    drone_ofvs.append(drone_kwh)
                    jump += 1
                elif route[idx][1] == ONLY_TRUCK:
                    truck_energy_consumption = data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]] * data["energy_kwh/km_t"]
                    total_energy_consumption += truck_energy_consumption
                    jump = 0
                    truck_kwh -= truck_energy_consumption
                    truck_ofvs.append(truck_kwh)
                    drone_ofvs.append(drone_kwh)
                    flee += 1
                elif route[idx][1] in [IDLE, FLY]:
                    truck_energy_consumption = data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]] * data["energy_kwh/km_t"]
                    total_energy_consumption += truck_energy_consumption
                    jump = 0
                    truck_kwh -= truck_energy_consumption
                    for iter in range(idx + 1, len(route)):
                        if (route[iter][1] != IDLE):
                            last = False
                            break
                        else:
                            last = True
                    if idx == len(route) - 1:
                        last = True
                    #드론 에너지 충전
                    charging_energy = (data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]] / data["speed_d"])/60 * (data["charging_kw_d"])
                    if drone_kwh < data["battery_kwh_d"] and not last: # 만땅이 아니고 마지막 운행이 아니라면
                        drone_kwh += charging_energy #kwh 단위 고려
                        if drone_kwh > data["battery_kwh_d"]:
                            drone_kwh = data["battery_kwh_d"] #충전해도 최대 배터리 양은 넘을 수 없음
                    truck_ofvs.append(truck_kwh)
                    drone_ofvs.append(drone_kwh)
            soc_t = [x / data["battery_kwh_t"] * 100 for x in truck_ofvs]
            soc_d = [x / data["battery_kwh_d"] * 100 for x in drone_ofvs]
            truck_soc.append(soc_t)
            drone_soc.append(soc_d)

        return truck_soc, drone_soc, total_energy_consumption

    def charging_kwh(self):
        routes_d = []
        ofvs = []
        charging_kwh=0
        for route in self.routes:
            #drone_path = [value[0] for value in route if value[1] != ONLY_TRUCK]
            start_index = 0
            first = True
            truck_drive_only = True
            truck_drive_with_drone = False
            drone_current_kwh = data["battery_kwh_d"]
            drone_distance = 0
            drone_idle_distance = 0
            ofv = []
            customer_by_drone = [customer[0] for customer in route]
            for idx, customer in enumerate(route):
                # 맨처음 반영해주기 위해 추가
                last_condition = (idx == len(route) - 1) and truck_drive_with_drone
                if customer[1] == FLY and first:
                    start_index = idx
                    ofv = [drone_current_kwh] * (idx+1)
                    first = False
                # 얘까지는 ofv 그대로 채우기
                elif (customer[1] == CATCH and truck_drive_only):
                    # <드론이 미션 수행하는 케이스>, 이 인덱스까지 모두 더해준다
                    end_index = idx
                    for k in range(start_index, end_index):
                        if route[k][1] == ONLY_TRUCK:
                            ofv.append(drone_current_kwh)
                            continue
                        drone_distance = data["edge_km_d"][route[k][0]][route[k+1][0]]
                        energy_consumption = drone_distance * data["energy_kwh/km_d"]
                        drone_current_kwh -= energy_consumption
                        ofv.append(drone_current_kwh)

                    truck_drive_only = False
                    truck_drive_with_drone = True
                    start_index = idx
                elif customer[1] == FLY and truck_drive_with_drone: 
                    # <드론이 업혀있는 케이스>, 이 인덱스까지 모두 더해준다
                    end_index = idx
                    for k in range(start_index, end_index):
                        drone_idle_distance = data["edge_km_t"][route[k][0]][route[k+1][0]]
                        #energy_consumption = drone_idle_distance * data["energy_kwh/km_t"]
                        drone_idle_time = drone_idle_distance / data["speed_d"]
                        #drone_current_kwh -= energy_consumption
                        if drone_current_kwh < data["battery_kwh_d"]: # 만땅이 아니라면
                            drone_current_kwh += (data["charging_kw_d"]) * (drone_idle_time/60) #kwh 단위 고려
                            charging_kwh += (data["charging_kw_d"]) * (drone_idle_time/60)
                            if drone_current_kwh > data["battery_kwh_d"]:
                                drone_current_kwh = data["battery_kwh_d"] #충전해도 최대 배터리 양은 넘을 수 없음
                                charging_kwh += 0
                        ofv.append(drone_current_kwh)
                    truck_drive_with_drone = False
                    truck_drive_only = True
                    start_index = idx
                elif last_condition:
                    ofv.extend([drone_current_kwh] * (idx - end_index))
                # 정말 한 라우트의 모든 비짓 타입이 꽁일 경우도 생각해줘야할듯 
                elif(idx == len(route) - 1) and truck_drive_only:
                    ofv = [drone_current_kwh] * (idx+1)
            soc = [x / data["battery_kwh_d"] * 100 for x in ofv]
            ofvs.append(soc)
            routes_d.append(customer_by_drone)
        return charging_kwh    

    
    def time_arrival(self):
        """Time arrival of Truck

        Returns:
            _type_: _description_
        """
        time_arrival = []
        for route in self.routes:
            truck_time = 0
            drone_time = 0
            time_table = [0]
            jump = 0
            flee = 0
            for idx in range(1, len(route)):
                if route[idx][1] == CATCH or ((route[idx][1] == FLY) and (route[idx-1][1] in [ONLY_TRUCK, ONLY_DRONE])):
                    #max
                    drone_time += data["edge_km_d"][route[idx-flee-1][0]][route[idx][0]]/data["speed_d"]
                    truck_time += data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]]/data["speed_t"]
                    longer_time = max(drone_time, truck_time)
                    time_table.append(longer_time)
                    drone_time = longer_time
                    truck_time = longer_time
                    flee = 0
                    jump = 0
                elif route[idx][1] == ONLY_DRONE:
                    drone_time += data["edge_km_d"][route[idx-flee-1][0]][route[idx][0]]/data["speed_d"]
                    time_table.append(drone_time)
                    jump += 1
                elif route[idx][1] == ONLY_TRUCK:
                    truck_time += data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]]/data["speed_t"]
                    time_table.append(truck_time)
                    flee += 1
                elif route[idx][1] in [IDLE, FLY]:
                    truck_time += data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]]/data["speed_t"]
                    time_table.append(truck_time)
                    drone_time = truck_time
        
            time_arrival.append(time_table)
        return time_arrival
    
    def renew_time_arrival(self):
        truck_time_arrivals = []  # 각 route의 트럭 도착 시간을 담은 리스트
        drone_time_arrivals = []  # 각 route의 드론 도착 시간을 담은 리스트

        for route in self.routes:
            truck_time_arrivals.append(self.new_time_arrival_per_route(route)['eTruck'])
            drone_time_arrivals.append(self.new_time_arrival_per_route(route)['eVTOL'])

        return truck_time_arrivals, drone_time_arrivals

    def new_time_arrival(self):
  
        truck_time_arrivals = []  # 각 route의 트럭 도착 시간을 담은 리스트
        drone_time_arrivals = []  # 각 route의 드론 도착 시간을 담은 리스트

        for route in self.routes:
            truck_time = 0
            drone_time = 0
            truck_time_table = [0]  # 트럭 도착 시간을 담을 리스트
            drone_time_table = [0]  # 드론 도착 시간을 담을 리스트
            jump = 0
            flee = 0

            for idx in range(1, len(route)):
                if route[idx][1] == CATCH and route[idx][0] == 0:
                    drone_time += data["edge_km_d"][route[idx-flee-1][0]][route[idx][0]] / data["speed_d"]
                    truck_time += data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]] / data["speed_t"]
                    truck_time_table.append(truck_time)
                    drone_time_table.append(drone_time)
                    flee = 0
                    jump = 0

                elif route[idx][1] == CATCH and route[idx][0] != 0:
                    drone_time += data["edge_km_d"][route[idx-flee-1][0]][route[idx][0]] / data["speed_d"]
                    truck_time += data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]] / data["speed_t"]
                    longer_time = max(drone_time, truck_time)
                    truck_time_table.append(longer_time)
                    drone_time_table.append(longer_time)
                    drone_time = longer_time
                    truck_time = longer_time
                    truck_time += data["service_time"]
                    flee = 0
                    jump = 0

                elif (route[idx][1] == FLY) and (route[idx-1][1] in [ONLY_TRUCK, ONLY_DRONE]):
                    drone_time += data["edge_km_d"][route[idx-flee-1][0]][route[idx][0]] / data["speed_d"]
                    truck_time += data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]] / data["speed_t"]
                    longer_time = max(drone_time, truck_time)
                    truck_time_table.append(longer_time)
                    drone_time_table.append(longer_time)
                    drone_time = longer_time
                    truck_time = longer_time
                    truck_time += data["service_time"]
                    flee = 0
                    jump = 0

                elif (route[idx][1] in [IDLE, FLY]) and (route[idx-1][1] not in [ONLY_TRUCK, ONLY_DRONE]):
                    truck_time += data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]] / data["speed_t"]
                    truck_time_table.append(truck_time)
                    drone_time = truck_time
                    drone_time_table.append(drone_time)
                    truck_time += data["service_time"]

                elif route[idx][1] == ONLY_DRONE:
                    drone_time += data["edge_km_d"][route[idx-flee-1][0]][route[idx][0]] / data["speed_d"]
                    drone_time_table.append(drone_time)
                    drone_time += data["takeoff_landing_time"]
                    truck_time_table.append(None)
                    jump += 1
                elif route[idx][1] == ONLY_TRUCK:
                    truck_time += data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]] / data["speed_t"]
                    truck_time_table.append(truck_time)
                    truck_time += data["service_time"]
                    drone_time_table.append(None)
                    flee += 1

            truck_time_arrivals.append(truck_time_table)  # 현재 route의 트럭 도착 시간을 전체 리스트에 추가
            drone_time_arrivals.append(drone_time_table)  # 현재 route의 드론 도착 시간을 전체 리스트에 추가

        return truck_time_arrivals, drone_time_arrivals

    def new_time_arrival_per_route(self, route):
        truck_time = 0
        drone_time = 0
        waiting_time = []
        truck_time_table = {'source': 0}  # 트럭 도착 시간을 담을 리스트
        drone_time_table = {'source': 0}  # 드론 도착 시간을 담을 리스트
        jump = 0
        flee = 0
        for idx in range(1, len(route)):
            catch = route[idx][1] == CATCH and route[idx][0] != 0
            catch_at_sink = route[idx][1] == CATCH and route[idx][0] == 0
            catch_with_fly = (route[idx][1] == FLY) and (route[idx-1][1] in [ONLY_TRUCK, ONLY_DRONE])
            if catch_at_sink:
                drone_time += data["edge_km_d"][route[idx-flee-1][0]][route[idx][0]] / data["speed_d"]
                truck_time += data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]] / data["speed_t"]
                truck_time_table.update({route[idx][0] : truck_time})
                drone_time_table.update({route[idx][0] : drone_time})
                flee = 0
                jump = 0

            elif catch or catch_with_fly:
                drone_time += data["edge_km_d"][route[idx-flee-1][0]][route[idx][0]] / data["speed_d"]
                truck_time += data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]] / data["speed_t"]
                longer_time = max(drone_time, truck_time)
                waiting_time.append(abs(drone_time - truck_time))
                truck_time_table.update({route[idx][0] : longer_time})
                drone_time_table.update({route[idx][0] : longer_time})
                drone_time = longer_time
                truck_time = longer_time
                truck_time += data["service_time"]
                flee = 0
                jump = 0

            elif (route[idx][1] in [IDLE, FLY]) and (route[idx-1][1] not in [ONLY_TRUCK, ONLY_DRONE]):
                truck_time += data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]] / data["speed_t"]
                truck_time_table.update({route[idx][0] : truck_time})
                drone_time = truck_time
                drone_time_table.update({route[idx][0] : drone_time})
                truck_time += data["service_time"]

            elif route[idx][1] == ONLY_DRONE:
                drone_time += data["edge_km_d"][route[idx-flee-1][0]][route[idx][0]] / data["speed_d"]
                drone_time_table.update({route[idx][0] : drone_time})
                drone_time += data["takeoff_landing_time"]
                truck_time_table.update({route[idx][0] : None})
                jump += 1
            elif route[idx][1] == ONLY_TRUCK:
                truck_time += data["edge_km_t"][route[idx-jump-1][0]][route[idx][0]] / data["speed_t"]
                truck_time_table.update({route[idx][0] : truck_time})
                truck_time += data["service_time"]
                drone_time_table.update({route[idx][0] : None})
                flee += 1

        return {'eTruck' : truck_time_table, 'eVTOL' : drone_time_table, 'Over Waiting Time' : any(time > data["max_waiting_time"] for time in waiting_time)}

    def truck_soc(self):
        routes_t = []
        ofvs = []
        for route in self.routes:
            #truck_path = [value for value in route if value[1] != ONLY_DRONE]
            start_index = 0
            truck_drive_only = True
            truck_drive_with_drone = False
            truck_current_kwh = data["battery_kwh_t"]
            truck_distance = 0
            truck_idle_distance = 0
            start_index = 0
            ofv = [truck_current_kwh]
            customer_by_truck = [customer[0] for customer in route]
            for idx, customer in enumerate(route):
                last_condition = (idx == len(route) - 1) and truck_drive_with_drone
                if (customer[1] == CATCH and truck_drive_only) or last_condition:
            
                    ### 이 인덱스까지 모두 더해준다
                    end_index = idx
                    for k in range(start_index, end_index):
                        if route[k][1] == ONLY_DRONE:
                            ofv.append(truck_current_kwh)
                            continue
                        truck_distance = data["edge_km_t"][route[k][0]][route[k+1][0]]
                        energy_consumption = truck_distance * data["energy_kwh/km_t"]
                        truck_current_kwh -= energy_consumption
                        ofv.append(truck_current_kwh)

                    truck_drive_only = False
                    truck_drive_with_drone = True
                    start_index = idx
                
                elif customer[1] == FLY and truck_drive_with_drone: 
                    ### 이 인덱스까지 모두 더해준다
                    end_index = idx
                    for k in range(start_index, end_index):
                        truck_idle_distance = data["edge_km_t"][route[k][0]][route[k+1][0]]
                        energy_consumption = truck_idle_distance * data["energy_kwh/km_t"]
                        truck_idle_time = truck_idle_distance / data["speed_t"]
                        truck_current_kwh -= energy_consumption
                        truck_current_kwh -= (data["charging_kw_d"]) * (truck_idle_time/60)
                        ofv.append(truck_current_kwh)
                    truck_drive_with_drone = False
                    truck_drive_only = True
                    start_index = idx
                elif (idx == len(route) - 1) and truck_drive_only:
                    ofv = [data["battery_kwh_t"]] * len(route)
            soc = [x / data["battery_kwh_t"] * 100 for x in ofv]
            ofvs.append(soc)
            routes_t.append(customer_by_truck)
        return routes_t, ofvs
    
    def objective_time_penalty(self):
        """
        energy_consumption + waiting time penalty
        """
        divided_routes = apply_dividing_route_to_routes(self.routes)
        truck_time_cost = 0.0
        truck_total_time = 0.0
        drone_time_cost = 0.0

        for route_info in divided_routes:
            vtype = route_info['vtype']
            path = route_info['path']
            route_info['time'] = []

            if vtype == 'truck':
                for i in range(len(path) - 1):
                    loc_from = path[i][0] if isinstance(path[i], tuple) else path[i]
                    loc_to = path[i+1][0] if isinstance(path[i+1], tuple) else path[i+1]

                    edge_weight = data["edge_km_t"][loc_from][loc_to]
                    truck_total_time += edge_weight / data["speed_t"]
                for j in range(len(path)):
                    if path[j][1] == 1:
                        start_index = j
                    elif path[j][1] == 3 and start_index is not None:
                        for k in range(start_index, j):
                            time_cost = data["edge_km_t"][path[k][0]][path[k+1][0]]
                            truck_time_cost += time_cost / data["speed_t"]
                        start_index = None
                        route_info['time'].append(truck_time_cost)
                        truck_time_cost = 0.0


            elif vtype == 'drone': 
                start_index = None
                flag = 0
                for j in range(len(path)):
                    if flag == 0 and path[j][1] == 1:
                        start_index = j
                        flag = 1
                    elif path[j][1] == 3 and start_index is not None:
                        for k in range(start_index, j):
                            time_cost = data["edge_km_d"][path[k][0]][path[k+1][0]]
                            drone_time_cost += time_cost / data["speed_d"]
                        flag = 0
                        start_index = None
                        route_info['time'].append(drone_time_cost)
                        drone_time_cost = 0.0

        grouped_paths = {k: [item['time'] for item in g] for k, g in groupby(divided_routes, key=lambda x: x['vid'])}

        waiting_time = {k: [abs(x - y) for x, y in zip(*v)] for k, v in grouped_paths.items() if len(v) == 2}

        total_sum = sum(sum(values) for values in waiting_time.values())

        return total_sum
    
    def object_total_timee(self):
        total_time_of_routes = []
        waiting_time_of_routes = []
        for route in self.routes:
            total_time = 0
            drone_time = 0
            truck_time = 0
            waiting_time = 0
            flag = 0
            drone_distance = 0
            truck_distance = 0
            for i in range(len(route)-1):
                if route[i][1] == IDLE or None:
                    total_time += (data["edge_km_t"][route[i][0]][route[i+1][0]])/data["speed_t"]
                
                elif flag == 0 and route[i][1] == FLY:
                    start_index = i
                    flag = 1
                    j = i + 1
                    while flag == 1 and j < len(route):
                        if route[j][1] == FLY:
                            flag = 0
                            end_index = j
                        elif route[j][1] == CATCH:
                            flag = 0
                            end_index = j
                        else:
                            flag = 1
                        j += 1

                    # 이 start_index와 end_index 사이의 drone_path와 truck_path를 추출하고, 각 subroute의 소요시간 계산
                    drone_path = [value for value in route[start_index:end_index + 1] if value[1] != ONLY_TRUCK]  
                    truck_path = [value for value in route[start_index:end_index + 1] if value[1] != ONLY_DRONE]  

                    ##  end_index + 1 assert 필요
                    
                    for l in range(len(drone_path)-1):
                        drone_distance += data["edge_km_t"][drone_path[l][0]][drone_path[l+1][0]]
                        
                    for m in range(len(truck_path)-1):    
                        truck_distance += data["edge_km_d"][truck_path[m][0]][truck_path[m+1][0]]        
                    
                    drone_time = drone_distance / data["speed_d"] + (len(drone_path)-2)*data["service_time"]
                    truck_time = truck_distance / data["speed_t"] + (len(truck_path)-1)*data["service_time"]
                    
                    if drone_time >= truck_time:
                        total_time += drone_time
                        waiting_time += (drone_time - truck_time)
                    else:
                        total_time += truck_time
                        waiting_time += (truck_time - drone_time)
                    
                    drone_time = 0
                    truck_time = 0
                elif route[i][1] == CATCH:
                    total_time += (data["edge_km_t"][route[i][0]][route[i+1][0]])/data["speed_t"]
            total_time_of_routes.append(total_time)
            waiting_time_of_routes.append(waiting_time)
        return sum(total_time_of_routes) + sum(waiting_time_of_routes)
    
    def objective_total_time(self):
        alpha = 1
        total_time_of_routes = []   # list of list
        waiting_time_of_routes = [] # list of list
        for route in self.routes:
            total_time_list, waiting_time_list = self.calculate_time_per_route(route)
            total_time_of_routes.append(total_time_list)
            waiting_time_of_routes.append(waiting_time_list)
        return sum(sum(sublist) for sublist in total_time_of_routes) + alpha * sum(sum(sublist) for sublist in waiting_time_of_routes)
    
    def calculate_time_per_route(self, route):
        total_time = 0
        total_time_list = [0] * len(route)
        drone_time = 0
        truck_time = 0
        waiting_time = 0
        waiting_time_list = []
        flag = 0
        drone_distance = 0
        truck_distance = 0
        for i in range(len(route)-1):
            if route[i][1] == IDLE or None:
                total_time += (data["edge_km_t"][route[i][0]][route[i+1][0]])/data["speed_t"]
                total_time_list[i+1] = total_time

            elif flag == 0 and route[i][1] == FLY:
                # FLY로 시작하는 index를 발견한다면 end_index까지 찾고 드론, 트럭 각각의 time_cost 계산
                start_index = i
                flag = 1
                j = i + 1
                while flag == 1 and j < len(route):
                    if route[j][1] == FLY:
                        flag = 0
                        end_index = j
                    elif route[j][1] == CATCH:
                        flag = 0
                        end_index = j
                    else:
                        flag = 1
                    j += 1
                # end_index 발견

                # start_index와 end_index 사이의 drone_path와 truck_path를 추출
                drone_path = [value for value in route[start_index:end_index + 1] if value[1] != ONLY_TRUCK]  
                truck_path = [value for value in route[start_index:end_index + 1] if value[1] != ONLY_DRONE]  

                ##  end_index + 1 assert 필요
                
                # 각 subroute의 소요시간 계산
                for l in range(len(drone_path)-1):
                    drone_distance += data["edge_km_t"][drone_path[l][0]][drone_path[l+1][0]]
                    
                for m in range(len(truck_path)-1):    
                    truck_distance += data["edge_km_d"][truck_path[m][0]][truck_path[m+1][0]]        
                
                drone_time = drone_distance / data["speed_d"] + (len(drone_path)-2)*data["takeoff_landing_time"]
                truck_time = truck_distance / data["speed_t"] + (len(truck_path)-1)*data["service_time"]
                
                if drone_time >= truck_time:
                    total_time += drone_time
                    waiting_time += (drone_time - truck_time)
                else:
                    total_time += truck_time
                    waiting_time += (truck_time - drone_time)
                if end_index != (len(route) - 1):
                    waiting_time_list.append(waiting_time)
                waiting_time = 0
                drone_time = 0
                truck_time = 0

                total_time_list[end_index] = total_time
                # 아직 중간 사이사이 index는 꽁임

            elif route[i][1] == CATCH:
                total_time += (data["edge_km_t"][route[i][0]][route[i+1][0]])/data["speed_t"]
                total_time_list[i+1] = total_time

        return total_time_list, waiting_time_list

    def cost(self):
        """
        Alias for objective method. Used for plotting.
        """
        return self.objective()
    

    def find_route(self, customer):
       
        for route in self.routes['route']:
            if customer in route['path']:
                return route
            
        raise ValueError(f"Solution does not contain customer {customer}.")
    
