import numpy as np
import random
import numpy.linalg as LA
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Agent():
    def __init__(self, id, pos):
        self.id = id
        self.pos = pos
        self.path = []
        # self.path_pos = []
        self.waiting = []
        self.timestamp = []
        self.vel = 100

    def move(self):
        self.pos = (0,0)

class Task():
    def __init__(self, id, pos):
        self.id = id
        self.pos = pos
        self.agent = -1
        self.duration = 5
        self.start = -1
        self.end = -1
        self.const = []
        self.reward = 10000000
        self.done = 0

class Constraint():
    def __init__(self, id, operator, task_1, tp):
        self.id = id
        self.operator = operator
        # operator
        # 0 : time_before
        # 1 : time_after
        # 2 : before
        # 3 : after
        # 4 : simultaneous
        # 5 : during_start
        # 6 : during_end
        self.t1 = task_1
        self.t2 = []
        self.time = []
        if operator < 2:
            self.time = tp
        else:
            self.t2 = tp

class Info_search():
    def __init__(self, Agents, Tasks):
        self.Agents = Agents
        self.Tasks = Tasks
        self.N_Agents = len(self.Agents)
        self.N_Tasks = len(self.Tasks)
        self.assigned = []

    def Cal_max_path(self):
        tp = []
        for i in range(self.N_Agents):
            tp.append(len(self.Agents[i].path))
        return max(tp)

    def Cal_agent_time(self, agent_id):
        if len(self.Agents[agent_id].timestamp) > 0:
            tt = self.Agents[agent_id].timestamp[-1] + self.Tasks[self.Agents[agent_id].path[-1]].duration
        else:
            tt = 0
        return tt

    def Cal_makespan(self):
        tp = []
        for i in range(self.N_Agents):
            if len(self.Agents[i].timestamp) > 0:
                tp.append(max(self.Agents[i].timestamp))
            else:
                tp.append(0)
        return max(tp)

    def Cal_time(self, agent_id):
        self.Agents[agent_id].timestamp = []
        for j in range(len(self.Agents[agent_id].path)):
            path_cur = self.Agents[agent_id].path[j]
            if j == 0:
                self.Tasks[path_cur].start = \
                    LA.norm(np.asarray(self.Tasks[path_cur].location) - np.asarray(self.Agents[agent_id].location)) / \
                    self.Agents[agent_id].vel
            else:
                path_past = self.Agents[agent_id].path[j - 1]
                self.Tasks[path_cur].start = \
                    self.Tasks[path_past].end + \
                    LA.norm(np.asarray(self.Tasks[path_cur].location) - np.asarray(self.Tasks[path_past].location)) / \
                    self.Agents[agent_id].vel
            self.Tasks[path_cur].start += self.Agents[agent_id].waiting[j]
            self.Tasks[path_cur].end = self.Tasks[path_cur].start + self.Tasks[path_cur].duration
            self.Agents[agent_id].timestamp.append(self.Tasks[path_cur].start)

    def Task_insert(self, agent_id, task_id, loc):
        self.Agents[agent_id].path.insert(loc, task_id)
        self.Agents[agent_id].waiting.insert(loc, 0)
        self.Tasks[task_id].agent = agent_id
        self.assigned.append(task_id)
        self.Cal_time(agent_id)

    def Task_remove(self, agent_id, task_id):
        # if task_id in self.Agents[agent_id].path:
        ind = self.Agents[agent_id].path.index(task_id)
        self.Agents[agent_id].path.pop(ind)
        self.Agents[agent_id].waiting.pop(ind)
        self.assigned.pop(self.assigned.index(task_id))
        self.Tasks[task_id].start = -1
        self.Tasks[task_id].end = -1
        self.Tasks[task_id].agent = -1
        self.Agents[agent_id].timestamp.pop(ind)
        self.Cal_time(agent_id)

    def Cal_score(self, agent_id, task_id, path_id):
        before_agent_max = self.Cal_agent_time(agent_id)
        before_total_max = self.Cal_makespan()
        self.Task_insert(agent_id, task_id, path_id)
        after_agent_max = self.Cal_agent_time(agent_id)
        after_total_max = self.Cal_makespan()
        self.Task_remove(agent_id, task_id)
        score = self.Tasks[task_id].reward \
                - (after_total_max - before_total_max)*0.05\
                - (after_agent_max - before_agent_max)

        score = max(score, 0)
        return score

    def Task_alloc(self):
        for tt in range(self.N_Tasks):
            Score_mat = np.zeros((self.N_Tasks, self.N_Agents, self.Cal_max_path()+1))
            for i in range(self.N_Tasks):
                if not i in self.assigned:
                    for j in range(self.N_Agents):
                        if len(self.Agents[j].path) > 0:
                            for k in range(min(len(self.Agents[j].path)+1,1)):
                                aa = len(self.Agents[j].path) - k
                                Score_mat[i, j, aa] = self.Cal_score(j, i, aa)
                        else:
                            Score_mat[i, j, 0] = self.Cal_score(j, i, 0)
            if sum(sum(sum(Score_mat))) > 0:
                ind = np.unravel_index(np.argmax(Score_mat, axis=None), Score_mat.shape)
                self.Task_insert(ind[1], ind[0], ind[2])
                print(ind)
        for i in range(self.N_Agents):
            self.Cal_time(i)

def clamp(angle):
    if angle > np.pi:
        angle -= 2 * np.pi
    elif angle < -np.pi:
        angle += 2 * np.pi
    return angle

def gen_triangle(center, ang_):
    mat_ = np.zeros((4, 2))
    mat_[0] = [np.sin(ang_ - np.pi / 2), np.cos(ang_ - np.pi / 2)] + center
    mat_[1] = [3.0 * np.sin(ang_), 3.0 * np.cos(ang_)] + center
    mat_[2] = [np.sin(ang_ + np.pi / 2), np.cos(ang_ + np.pi / 2)] + center
    mat_[3] = [np.sin(ang_ - np.pi / 2), np.cos(ang_ - np.pi / 2)] + center
    return mat_

class Agent_(): # for animation
    def __init__(self, agent_id, agent_type, x, y):
        self.id = agent_id
        self.type = agent_type
        self.x = x
        self.y = y
        self.pos = np.array([x, y])
        self.yaw = 0
        if self.type == 0:
            self.vel = 0.5
            self.max_ang = 10
        elif self.type == 1:
            self.vel = 1
            self.max_ang = 100
        self.phase = -1
        self.target = [x, y]
        self.target_i = 100
        self.save_pos = [x, y]
        self.waypoints = []
        #self.waypoint_idx =
        self.task = -1
        self.task_type = -1
        self.capture = 0
        self.done_task = []
        self.done_task_fix = []
        self.ind = 0
        # Type 0 : USV
        # Type 1 : UAV

    def set_waypoint(self, wp):
        self.waypoints = wp

    def set_target(self):
        tp = np.asarray(self.target) - np.asarray(self.pos)
        if LA.norm(tp) < self.vel:
            self.ind += 1
            if self.ind > len(self.waypoints)-1:
                self.ind = 0
            # print(self.id, len(self.waypoints), self.ind)
            self.target = self.waypoints[self.ind]

    def set_target_direct(self, pos):
        self.target = pos

    def move(self):
        diff = np.asarray(self.target) - self.pos
        ang = np.arctan2(diff[0], diff[1])
        # print(ang, self.yaw)
        ang_diff = max(min(clamp(ang - self.yaw), self.max_ang*np.pi/180), -self.max_ang*np.pi/180)

        if LA.norm(diff) > self.vel:
            dist = min(LA.norm(diff), self.vel)

            self.yaw += ang_diff
            self.yaw = clamp(self.yaw)
            # print(self.location, np.array([np.sin(self.yaw*np.pi/180) * dist, np.cos(self.yaw*np.pi/180) * dist]))
            self.pos += np.array([np.sin(self.yaw) * dist, np.cos(self.yaw) * dist])
            # print(self.location)
            self.x = self.pos[0]
            self.y = self.pos[1]
            # print(self.x)
            self.save_pos.append([self.x, self.y])

class Illegal_Agent(): # for animation
    def __init__(self, agent_id, x, y):
        self.id = agent_id
        self.x = x
        self.y = y
        self.pos = np.array([x, y])
        self.yaw = 0
        self.vel = 0.4
        self.phase = 0
        self.fishing_time = 0
        self.target = [x, y]
        self.save_pos = [x, y]
        self.in_track = 0
        self.in_capture = 0

    # phase 0 : 조업하기 / phase 1 : USV 한테 발견되면 일단 도망 / phase 2 : 충분히 가까워지면 끌려가기
    def set_target(self, blue):
        if self.phase == 0:
            if self.fishing_time > 100:
                self.fishing_time = 0
                self.target = self.pos + [random.random()*2-1, random.random()*2-1]
            if LA.norm(np.asarray(self.target) - self.pos) < 0.5:
                self.fishing_time += 1
        elif self.phase == 1:
            self.target = 2*self.pos-blue
            print(self.target)
        elif self.phase == 2:
            self.target = blue
            self.vel = 0.5

    def move(self):
        diff = np.asarray(self.target) - self.pos
        ang = np.arctan2(diff[0], diff[1])
        # print(ang, self.yaw)
        ang_diff = max(min(clamp(ang - self.yaw), 50*np.pi/180), -50*np.pi/180)

        if LA.norm(diff) > self.vel:
            dist = min(LA.norm(diff), self.vel)

            self.yaw += ang_diff
            self.yaw = clamp(self.yaw)
            # print(self.location, np.array([np.sin(self.yaw*np.pi/180) * dist, np.cos(self.yaw*np.pi/180) * dist]))
            self.pos += np.array([np.sin(self.yaw) * dist, np.cos(self.yaw) * dist])
            # print(self.location)
            self.x = self.pos[0]
            self.y = self.pos[1]
            # print(self.x)
            self.save_pos.append([self.x, self.y])

class Info_anim():
    def __init__(self, Agents, I_Agents, UAV_base, USV_base):
        self.Agents = Agents
        self.N_agents = len(Agents)
        self.N_agents0 = 0
        self.N_agents1 = 0
        for i in range(self.N_agents):
            if self.Agents[i].type == 0:
                self.N_agents0 += 1
            elif self.Agents[i].type == 1:
                self.N_agents1 += 1
        # Type 0 : USV
        # Type 1 : UAV
        self.N_tasks = 0
        self.I_Agents = I_Agents
        self.N_i_agents = len(I_Agents)
        self.Tasks = []
        self.UAV_base = UAV_base
        self.USV_base = USV_base
        self.trig = 0
        self.total_tasks = []
        self.mat = []
        self.reward_mat = [[0, 0, 1000, 1000],[100, 1000, 0, 0]]
        self.path_info = []
        self.check1 = 0
        self.check2 = 0

        self.fig = plt.figure()
        self.ax = plt.axes(xlim=(-10, 110), ylim=(-10, 110))
        self.line = []
        for i in range(len(Agents)):
            if self.Agents[i].type == 0:
                l1, = self.ax.fill([Agents[i].x-0.5, Agents[i].x+0, Agents[i].x+0.5],
                                   [Agents[i].y+0, Agents[i].y+1.5, Agents[i].y+0], 'b', lw=2)
            else:
                l1, = self.ax.fill([Agents[i].x - 0.5, Agents[i].x + 0, Agents[i].x + 0.5],
                                   [Agents[i].y + 0, Agents[i].y + 1.5, Agents[i].y + 0], 'g', lw=2)
            self.line.append(l1)
        for i in range(len(I_Agents)):
            l2, = self.ax.fill([I_Agents[i].x-1.5, I_Agents[i].x+1, I_Agents[i].x+1.5],
                               [I_Agents[i].y+0, I_Agents[i].y+1.5, I_Agents[i].y+0], 'r', lw=2)
            self.line.append(l2)

    def initialization(self, ss):
        if not len(ss) == self.N_agents1:
            # Type 0 : USV
            # Type 1 : UAV
            print("ERROR search task allocation number is different from UAV number")
        else:
            for i in range(len(ss)):
                self.Agents[i+self.N_agents0].waypoints = ss[i]
                # print(self.Agents[i+self.N_agents0].waypoint)

    def ani_init(self):
        return self.line

    def ani_update(self, i):

        N = len(self.Agents)
        for j in range(N):
            if self.Agents[j].type == 0: #USV
                if self.Agents[j].phase == -1:
                    self.Agents[j].phase = 0
                elif self.Agents[j].phase == 0:
                    self.Agents[j].set_target_direct(self.USV_base)
                elif self.Agents[j].phase == 1:
                    self.Agents[j].set_target_direct(self.I_Agents[self.Agents[j].target_i].location)
                    self.Agents[j].move()
                    self.line[j].set_xy(gen_triangle(self.Agents[j].location, self.Agents[j].yaw))
                    if LA.norm(self.Agents[j].location - self.I_Agents[self.Agents[j].target_i].location) < 5:
                        self.Agents[j].phase = 2
                        for k in range(self.N_agents1):
                            # print(LA.norm(self.Agents[j].location - self.Agents[k+self.N_agents0].location), k+self.N_agents0)
                            if LA.norm(self.Agents[j].location - self.Agents[k + self.N_agents0].location) < 10:
                                self.Agents[k+self.N_agents0].phase = -1
                elif self.Agents[j].phase == 2:
                    self.Agents[j].set_target_direct(self.USV_base)
                    self.Agents[j].move()
                    self.line[j].set_xy(gen_triangle(self.Agents[j].location, self.Agents[j].yaw))
                    if LA.norm(self.Agents[j].location - self.USV_base) < 5:
                        self.Agents[j].phase = 0

            elif self.Agents[j].type == 1: #UAV
                if self.Agents[j].phase == -1:
                    self.Agents[j].phase = 0
                elif self.Agents[j].phase == 0:
                    # print(self.Agents[j].waypoints)
                    self.Agents[j].set_target()
                    self.Agents[j].move()
                    self.line[j].set_xy(gen_triangle(self.Agents[j].location, self.Agents[j].yaw))
                    tp = 0
                    rec = 0
                    for k in range(self.N_i_agents):
                        if LA.norm(self.Agents[j].location - self.I_Agents[k].location) < 5:
                            if self.I_Agents[k].phase == 0:
                                tp += 1
                                rec = k
                    if tp == 1:
                        print(j, self.Agents[j].phase, "발견했다", self.I_Agents[rec].phase)
                        tp1 = 0
                        for k in range(self.N_agents1):
                            if LA.norm(self.Agents[k+self.N_agents0].location - self.I_Agents[rec].location) < 5:
                                # 주변에 다른 UAV 없는지 체크 중
                                tp1 += 1
                                rec1 = k
                        if tp1 == 1:

                            self.Agents[j].phase = 1
                            tp2 = []
                            for k in range(self.N_agents0):
                                if self.Agents[k].phase == 0:
                                    tp2.append([LA.norm(self.Agents[k].location - self.I_Agents[rec].location), k])
                            if len(tp2) > 0:
                                tp_ind = tp2[np.argmin(np.transpose(tp2)[0])][1]
                                self.Agents[tp_ind].phase = 1
                                self.Agents[tp_ind].target_i = rec
                    # UAV 가 I_agent 를 만나고, 주변에 다른 UAV 가 없다면 phase 1 로 넘긴다
                elif self.Agents[j].phase == 1:
                    rec = -1
                    for k in range(len(self.I_Agents)):
                        if LA.norm(self.Agents[j].location - self.I_Agents[k].location) < 5:
                            rec = k
                    if not rec == -1:
                        tp = 0
                        for k in range(self.N_agents0):
                            if self.Agents[k].target_i == rec:
                                tp += 1
                        if tp == 0:
                            tp2 = []
                            for k in range(self.N_agents0):
                                if self.Agents[k].phase == 0:
                                    if not self.Agents[k].target_i == rec:
                                        tp2.append([LA.norm(self.Agents[k].location - self.I_Agents[rec].location), k])
                            if len(tp2) > 0:
                                tp_ind = tp2[np.argmin(np.transpose(tp2)[0])][1]
                                self.Agents[tp_ind].phase = 1
                                self.Agents[tp_ind].target_i = rec
                        self.Agents[j].set_target_direct(self.I_Agents[rec].location) # 만난 바로 그 agent 의 번호를 알아야 함..
                        self.Agents[j].move()
                        self.line[j].set_xy(gen_triangle(self.Agents[j].location, self.Agents[j].yaw))
                    # USV 가 I_agent 가까이 오면, 이제 다시 Phase 0 으로 넘긴다

        for j in range(len(self.I_Agents)):
            if self.I_Agents[j].phase == 0:
                tp = []
                for k in range(self.N_agents):
                    if self.Agents[k].type == 0:
                        if self.Agents[k].target_i == j:
                            tp.append(LA.norm(self.Agents[k].location - self.I_Agents[j].location))
                if len(tp) > 0:
                    if min(tp) < 5:
                        self.I_Agents[j].phase = 1
            elif self.I_Agents[j].phase == 1:
                tp = []
                for k in range(self.N_agents):
                    if self.Agents[k].type == 0:
                        if self.Agents[k].target_i == j:
                            tp.append(LA.norm(self.Agents[k].location - self.I_Agents[j].location))
                if len(tp) > 0:
                    if min(tp) < 5:
                        self.I_Agents[j].phase = 2

            if self.I_Agents[j].phase == 0:
                self.I_Agents[j].set_target([])
            else:
                tp = []
                tp_ind = []
                for k in range(self.N_agents):
                    if self.Agents[k].type == 0:
                        tp_ind.append(k)
                        tp.append(LA.norm(self.Agents[k].location - self.I_Agents[j].location))
                if LA.norm(self.I_Agents[j].location - self.USV_base) < 5:
                    self.I_Agents[j].set_target(self.USV_base)
                else:
                    self.I_Agents[j].set_target(self.Agents[tp_ind[np.argmin(tp)]].location)
            self.I_Agents[j].move()

            tp = []
            for k in range(len(self.Agents)):
                if self.Agents[k].type == 1:
                    tp.append(LA.norm(self.I_Agents[j].location - self.Agents[k].location))
            self.line[N+j].set_xy(gen_triangle(self.I_Agents[j].location, self.I_Agents[j].yaw))

        return self.line

    def animate(self):
        self.anim = animation.FuncAnimation(self.fig, self.ani_update, init_func=self.ani_init, frames=50, interval=20, blit=False)
        plt.show()
        print("Done")