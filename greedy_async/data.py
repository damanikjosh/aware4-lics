import pandas as pd
import numpy as np

import os
data_path = os.path.dirname(os.path.abspath(__file__)) + '/data/Density 0.02'


class Obj:
    def __init__(self, raw):
        self.id = int(raw['id_obj'].iloc[0])
        self.obj_sea = bool(raw['Obj_Sea'].iloc[0])
        self.obj_air = bool(raw['Obj_Air'].iloc[0])
        self.obj_land = bool(raw['Obj_Land'].iloc[0])
        self.location_poly = raw[['Location_Poly_X', 'Location_Poly_Y']].dropna().values.tolist()
        self.location_point = raw[['Location_Point_X', 'Location_Point_Y']].dropna().values.tolist()
        self.density = float(raw['Density'].iloc[0])


class Agent:
    def __init__(self, raw):
        self.id = int(raw[0])
        self.type = int(raw[1])
        self.base = int(raw[2])
        self.gear1 = bool(raw[3])
        self.gear2 = bool(raw[4])
        self.gear3 = bool(raw[5])
        self.gear4 = bool(raw[6])
        self.location = [float(raw[7]), float(raw[8])]
        self.speed = float(raw[9])
        self.energy_consumption = float(raw[10])
        self.remain_energy = float(raw[11])

    def __repr__(self):
        return '<Agent %d: (%.2f, %.2f)>' % (self.id, self.location[0], self.location[1])


class Base:
    def __init__(self, raw):
        self.id = int(raw[0])
        self.location = [float(raw[1]), float(raw[2])]
        self.is_sea = bool(raw[3])
        self.is_air = bool(raw[4])
        self.is_land = bool(raw[5])

    def __repr__(self):
        return '[%d: (%.2f, %.2f)]' % (self.id, self.location[0], self.location[1])


class Node:
    def __init__(self, id, location, types):
        self.id = id
        self.location = location
        self.types = types

    def __repr__(self):
        return '[%d: (%.2f, %.2f)]' % (self.id, self.location[0], self.location[1])

    def dist_to(self, other):
        return np.sqrt((self.location[0] - other.location[0]) ** 2 + (self.location[1] - other.location[1]) ** 2)


N_PARAMS = 9

obj1_file = data_path + '/Obj_1.csv'
obj1_num = (sum(1 for _ in open(obj1_file)) - 1) // N_PARAMS
obj1_raw = [pd.read_csv(obj1_file, skiprows=i * N_PARAMS + 1, nrows=N_PARAMS, index_col=0, header=None).T for i in range(obj1_num)]
obj1 = [Obj(o) for o in obj1_raw]

obj2_file = data_path + '/Obj_2.csv'
obj2_num = (sum(1 for _ in open(obj2_file)) - 1) // N_PARAMS
obj2_raw = [pd.read_csv(obj2_file, skiprows=i * N_PARAMS + 1, nrows=N_PARAMS, index_col=0, header=None).T for i in range(obj2_num)]
obj2 = [Obj(o) for o in obj2_raw]

obj3_file = data_path + '/Obj_3.csv'
obj3_num = (sum(1 for _ in open(obj3_file)) - 1) // N_PARAMS
obj3_raw = [pd.read_csv(obj3_file, skiprows=i * N_PARAMS + 1, nrows=N_PARAMS, index_col=0, header=None).T for i in range(obj3_num)]
obj3 = [Obj(o) for o in obj3_raw]

agents_file = data_path + '/Agent.csv'
agents_raw = pd.read_csv(agents_file).values.tolist()
agents_num = len(agents_raw)
agents = [Agent(agents_raw[i]) for i in range(agents_num)]

bases_file = data_path + '/Base.csv'
bases_raw = pd.read_csv(bases_file).values.tolist()
bases_num = len(bases_raw)
bases = [Base(bases_raw[i]) for i in range(bases_num)]

search_nodes = []
id = 0
for obj in obj1:
    for point in obj.location_point:
        types = []
        if obj.obj_air:
            types.append(1)
        if obj.obj_land:
            types.append(2)
        if obj.obj_sea:
            types.append(3)
        search_nodes.append(Node(id, point, types))
        id += 1

enemy_nodes = []
id = 0
for obj in obj2:
    for point in obj.location_point:
        types = []
        if obj.obj_air:
            types.append(1)
        if obj.obj_land:
            types.append(2)
        if obj.obj_sea:
            types.append(3)
        enemy_nodes.append(Node(id, point, types))
        id += 1

dest_nodes = []
id = 0
for obj in obj3:
    for point in obj.location_poly:
        types = []
        if obj.obj_air:
            types.append(1)
        if obj.obj_land:
            types.append(2)
        if obj.obj_sea:
            types.append(3)
        dest_nodes.append(Node(id, point, types))
        id += 1

for i, enemy in enumerate(enemy_nodes):
    min_dist = 1e10
    min_dest = None
    for dest in dest_nodes:
        dist = np.sqrt((enemy.location[0] - dest.location[0]) ** 2 + (enemy.location[1] - dest.location[1]) ** 2)
        if dist < min_dist:
            min_dist = dist
            min_dest = dest
    enemy.dest = min_dest.location
    enemy.vehicle_id = 9 + i


if __name__ == '__main__':
    print('search_nodes:')
    print(search_nodes)

    print('enemy_nodes:')
    print(enemy_nodes)

    print('dest_nodes:')
    print(dest_nodes)