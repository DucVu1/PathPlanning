"""

Implementation of artificial potential field pathfinding algorithm
Improve the artificial potential field to solve the inaccessibility problem, and the local minima problem"""
from Original_APF import APF, Vector2d
import matplotlib.pyplot as plt
import math
from matplotlib.patches import Circle
import random


def check_vec_angle(v1: Vector2d, v2: Vector2d):
    v1_v2 = v1.deltaX * v2.deltaX + v1.deltaY * v2.deltaY
    angle = math.acos(v1_v2 / (v1.length * v2.length)) * 180 / math.pi
    return angle


class APF_Improved(APF):
    def __init__(self, start: (), goal: (), obstacles: [], k_att: float, k_rep: float, rr: float,
                 step_size: float, max_iters: int, goal_threshold: float, is_plot=False):
        self.start = Vector2d(start[0], start[1])
        self.current_pos = Vector2d(start[0], start[1])
        self.goal = Vector2d(goal[0], goal[1])
        self.obstacles = [Vector2d(OB[0], OB[1]) for OB in obstacles]
        self.k_att = k_att
        self.k_rep = k_rep
        self.rr = rr 
        self.step_size = step_size
        self.max_iters = max_iters
        self.iters = 0
        self.goal_threashold = goal_threshold
        self.path = list()
        self.is_path_plan_success = False
        self.is_plot = is_plot
        self.delta_t = 0.01

    def repulsion(self):            
        rep = Vector2d(0, 0)  
        for obstacle in self.obstacles:
            # obstacle = Vector2d(0, 0)
            rob_to_obs = self.current_pos - obstacle #distance from robot and obstable
            rob_to_goal = self.goal -self.current_pos#distance from goal to position
            if (rob_to_obs.length > self.rr):  
                pass
            else:
                rep_1 = Vector2d(rob_to_obs.direction[0], rob_to_obs.direction[1]).mul(rob_to_goal.pow())* self.k_rep * (
                        1.0 / rob_to_obs.length - 1.0 / self.rr) / (rob_to_obs.length ** 2)  
                rep_2 = Vector2d(rob_to_goal.direction[0], rob_to_goal.direction[1]).mul(rob_to_goal) * self.k_rep * ((1.0 / rob_to_obs.length - 1.0 / self.rr) ** 2) 
                rep +=(rep_1+rep_2)
        return rep


if __name__ == '__main__':
    k_att, k_rep = 1.0, 0.8
    rr = 1
    step_size, max_iters, goal_threashold = .2, 1000, .2  # It takes 4.37s to find the path 1000 times with a step size of 0.5, and 21s for 1000 times with a step size of 0.1.
    step_size_ = 2

    start, goal = (0, 0), (15, 15)
    is_plot = True
    if is_plot:
        fig = plt.figure(figsize=(7, 7))
        subplot = fig.add_subplot(111)
        subplot.set_xlabel('X-distance: m')
        subplot.set_ylabel('Y-distance: m')
        subplot.plot(start[0], start[1], '*r')
        subplot.plot(goal[0], goal[1], '*r')
    # Obstacle setting and drawing
    obs = [[1, 0],[1,4], [2, 0],[2,4], [3, 0],[3,4],[4,0] ,[4, 4],[5,0], [5, 4], [6, 0],[6,4],[7,0], [7, 4], [8, 0],[8,4]]
    print('obstacles: {0}'.format(obs))
    for i in range(0):
        obs.append([random.uniform(2, goal[1] - 1), random.uniform(2, goal[1] - 1)])

    if is_plot:
        for OB in obs:
            circle = Circle(xy=(OB[0], OB[1]), radius=rr, alpha=0.3)
            subplot.add_patch(circle)
            subplot.plot(OB[0], OB[1], 'xk')
    # t1 = time.time()
    # for i in range(1000):

    # path plan
    if is_plot:
        apf = APF_Improved(start, goal, obs, k_att, k_rep, rr, step_size, max_iters, goal_threashold, is_plot)
    else:
        apf = APF_Improved(start, goal, obs, k_att, k_rep, rr, step_size, max_iters, goal_threashold, is_plot)
    apf.path_plan()
    if apf.is_path_plan_success:
        path = apf.path
        path_ = []
        i = int(step_size_ / step_size)
        while (i < len(path)):
            path_.append(path[i])
            i += int(step_size_ / step_size)

        if path_[-1] != path[-1]:  #add last point
            path_.append(path[-1])
        print('planed path points:{}'.format(path_))
        print('path plan success')
        if is_plot:
            px, py = [K[0] for K in path_], [K[1] for K in path_]  #Path point x coordinate list, y coordinate list
            subplot.plot(px, py, '^k')
            plt.show()
    else:
        print('path plan failed')
    # t2 = time.time()
    # print('寻路1000次所用时间:{}, 寻路1次所用时间:{}'.format(t2-t1, (t2-t1)/1000))
