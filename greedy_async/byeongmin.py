from greedy.class_import import *
import csv

# random.seed(10)

# for animation
# Info_anim
Agents = [] # USV, UAV 둘 다 넣어준다
I_Agents = [] # 적군 정보

# for search task gen
# Info_search
agents = [] # UAV 만 넣어주면 됨
tasks = [] # 모든 감시해야 할 점들

#######################################################
# 감시 영역 정보가 들어 있음
file_name = 'Obj_1.csv'
f = open(file_name, 'r')
rdr = csv.reader(f)
cnt = 0
Poly_X = []
Poly_Y = []
Point_X = []
Point_Y = []
for line in rdr:
    # print(line)
    if line[0] == 'Location_Poly_X':
        Poly_X.append([])
        for i in range(len(line)):
            if line[i].isdigit():
                Poly_X[cnt].append(float(line[i]))
    elif line[0] == 'Location_Poly_Y':
        Poly_Y.append([])
        for i in range(len(line)):
            if line[i].isdigit():
                Poly_Y[cnt].append(float(line[i]))
    elif line[0] == 'Location_Point_X':
        Point_X.append([])
        for i in range(len(line)-1):
            if len(line[i+1])>0:
                Point_X[cnt].append(float(line[i+1]))
    elif line[0] == 'Location_Point_Y':
        Point_Y.append([])
        for i in range(len(line) - 1):
            if len(line[i + 1]) > 0:
                Point_Y[cnt].append(float(line[i+1]))
        cnt += 1

fig = plt.figure()
ax = fig.subplots()
ax.fill(Poly_X[0], Poly_Y[0], 'b')
ax.fill(Poly_X[1], Poly_Y[1], 'b')
ax.plot(Point_X[0], Point_Y[0], 'g*')
ax.plot(Point_X[1], Point_Y[1], 'g*')

cnt = 0
for i in range(len(Point_X)):
    for j in range(len(Point_X[i])):
        tasks.append(Task(cnt,[Point_X[i][j], Point_Y[i][j]]))
        cnt += 1
    # Search Task 만드는 거

#######################################################
# 적군 정보가 들어 있음
file_name = 'Obj_2.csv'
f = open(file_name, 'r')
rdr = csv.reader(f)
cnt = 0
illegal_agent_xy = []
for line in rdr:
    # print(line)
    if line[0] == 'id_obj':
        illegal_agent_xy.append([])
    elif line[0] == 'Location_Point_X':
        illegal_agent_xy[cnt].append(float(line[1]))
    elif line[0] == 'Location_Point_Y':
        illegal_agent_xy[cnt].append(float(line[1]))
        cnt += 1

for i in range(len(illegal_agent_xy)):
    I_Agents.append(Illegal_Agent(i,illegal_agent_xy[i][0],illegal_agent_xy[i][1]))

for i in range(len(illegal_agent_xy)):
    ax.plot(illegal_agent_xy[i][0], illegal_agent_xy[i][1], 'r*')

#######################################################
# Base 정보가 들어 있음
file_name = 'Obj_3.csv'
f = open(file_name, 'r')
rdr = csv.reader(f)
cnt = 0
goal_xy = []
for line in rdr:
    # print(line)
    if line[0] == 'id_obj':
        goal_xy.append([])
    elif line[0] == 'Location_Poly_X':
        goal_xy[cnt].append(float(line[1]))
    elif line[0] == 'Location_Poly_Y':
        goal_xy[cnt].append(float(line[1]))
        cnt += 1

for i in range(1):
    ax.plot(goal_xy[i][0], goal_xy[i][1], 'k*')


Agents.append(Agent_(0, 0, goal_xy[1][0], goal_xy[1][1]))
Agents.append(Agent_(1, 0, goal_xy[1][0], goal_xy[1][1]))
Agents.append(Agent_(2, 1, goal_xy[0][0], goal_xy[0][1]))
Agents.append(Agent_(3, 1, goal_xy[0][0], goal_xy[0][1]))
Agents.append(Agent_(4, 1, goal_xy[0][0], goal_xy[0][1]))
agents.append(Agent(0, goal_xy[1]))
agents.append(Agent(1, goal_xy[1]))
agents.append(Agent(2, goal_xy[1]))
info_anim = Info_anim(Agents, I_Agents, goal_xy[0], goal_xy[1])
info_search = Info_search(agents, tasks)
info_search.Task_alloc()
search_task = []
for i in range(3):
    search_task.append([])
    for j in range(len(info_search.Agents[i].path)):
        search_task[i].append(info_search.Tasks[info_search.Agents[i].path[j]].location)
ax.plot(np.transpose(search_task[0])[0], np.transpose(search_task[0])[1], 'red')
ax.plot(np.transpose(search_task[1])[0], np.transpose(search_task[1])[1], 'orange')
ax.plot(np.transpose(search_task[2])[0], np.transpose(search_task[2])[1], 'yellow')
#plt.show()
info_anim.initialization(search_task)
info_anim.animate()