import matplotlib.pyplot as plt
import numpy as np
import random
import math
from scipy import interpolate
import time
import scipy.interpolate as si

# ============================
# Node Class
# ============================
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0

# ============================
# Parameters Class
# ============================
class Parameters:
    def __init__(self):
        self.start = [1.5, -1.5]
        self.goal = [1.5, 1.5]
        self.bounds = [-2, 2, -2, 2]
        self.obstacles = np.array([
            [1.0, -0.5, 0.25],
            [1.0, 0.5, 0.25],
            [-0.0, 0.0, 0.25],
            [1.0, 0.0, 0.25],
            [-0.0, 0.5, 0.25],
            [-0.0, -0.5, 0.25]
        ])
        self.robot_radius = 0.5
        self.dt = 0.1
        self.v = 0.2
        self.r0 = 0.2
        self.omega_min = -0.7
        self.omega_max = 0.7
        self.n_omega = 15
        self.prediction_horizon = 25
        self.pause = 0.001
        self.fps = 10
        self.R = 0.1
        self.r_buffer = self.R

# ============================
# Utility Functions
# ============================
# Calculates Euclidean distance between two nodes
def distance(n1, n2):
    return np.hypot(n1.x - n2.x, n1.y - n2.y)

# Samples a random node within the given bounds
def sample_free(xmin, xmax, ymin, ymax):
    return Node(random.uniform(xmin, xmax), random.uniform(ymin, ymax))

# Generates a new node towards target with a max extend length
def steer(from_node, to_node, extend_length):
    dist = distance(from_node, to_node)
    if dist <= extend_length:
        return to_node
    theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
    return Node(from_node.x + extend_length * math.cos(theta),
                from_node.y + extend_length * math.sin(theta))

# Checks for collisions along the edge between two nodes
def check_edge_collision(n1, n2, obstacles, robot_radius, step_size=0.05):
    dist = distance(n1, n2)
    steps = int(dist / step_size)
    if steps == 0:
        steps = 1
    for i in range(steps + 1):
        t = i / steps
        x = n1.x + t * (n2.x - n1.x)
        y = n1.y + t * (n2.y - n1.y)
        for (ox, oy, r) in obstacles:
            if np.hypot(x - ox, y - oy) <= r + robot_radius:
                return False
    return True

# Returns node in list closest to a random node
def get_nearest_node(node_list, rnd_node):
    return min(node_list, key=lambda node: distance(node, rnd_node))

# Gets all nodes within a radius of the new node
def get_near_nodes(node_list, new_node, radius):
    return [node for node in node_list if distance(node, new_node) <= radius]

# Selects the best parent from nearby nodes based on cost
def choose_parent(new_node, near_nodes, obstacles, robot_radius):
    if not near_nodes:
        return new_node
    min_cost = float('inf')
    best_node = None
    for node in near_nodes:
        t_node = steer(node, new_node, distance(node, new_node))
        if check_edge_collision(node, t_node, obstacles, robot_radius):
            cost = node.cost + distance(node, t_node)
            if cost < min_cost:
                min_cost = cost
                best_node = node
    if best_node:
        new_node.parent = best_node
        new_node.cost = min_cost
    return new_node

# Rewires the nearby nodes if a better path is found through new_node
def rewire(node_list, new_node, near_nodes, obstacles, robot_radius):
    for node in near_nodes:
        t_node = steer(new_node, node, distance(new_node, node))
        if check_edge_collision(new_node, t_node, obstacles, robot_radius):
            cost = new_node.cost + distance(new_node, t_node)
            if cost < node.cost:
                node.parent = new_node
                node.cost = cost

# Constructs the path by backtracking from the goal node to the start
def extract_path(goal_node):
    path = [[goal_node.x, goal_node.y]]
    node = goal_node
    while node.parent is not None:
        node = node.parent
        path.append([node.x, node.y])
    return path[::-1]



"""

Beginning  OF TBD Section

"""


# ============================
# RRT* Main Planner
# ============================
# Builds and returns a path from start to goal using RRT*
def rrt_star(start, goal, bounds, obstacles, robot_radius, max_iter=1000, step_size=0.1, goal_sample_rate=0.1):
    xmin, xmax, ymin, ymax = bounds
    start_node = Node(*start)
    goal_node = Node(*goal)
    node_list = [start_node]

    for i in range(max_iter):
        if random.random() < goal_sample_rate:
            #TODO
            # sample goal node
            rnd_node = ...
        else:
            #TODO:
            # sample free node , you can use sample_free utility function
            rnd_node = ...


        #TODO
        # get the neares node to the random node , you can use the get_nearest_node
        nearest_node = ...

        #TODO 
        #call the steer function to get a new_node , it generates a new node in the direction with a step_length limit
        new_node = ...

        if not check_edge_collision(nearest_node, new_node, obstacles, robot_radius):
            continue
        
        #TODO:
        #get neear by nodes around the new_nodes with a radius of 0.5, you can use the get_near nodes function
        near_nodes = ...

    
        new_node = choose_parent(new_node, near_nodes, obstacles, robot_radius)

        if new_node.parent is None:
            continue

        node_list.append(new_node)

        #TODO 
        #rewire the node_list  , you can use the rewire function
        rewire(...)

        if distance(new_node, goal_node) <= step_size and check_edge_collision(new_node, goal_node, obstacles, robot_radius):
            goal_node.parent = new_node
            goal_node.cost = new_node.cost + distance(new_node, goal_node)
            return extract_path(goal_node)

    return None

# ============================
# DWA Functions
# ============================
# Integrates robot dynamics forward by one step


def euler_integration(tspan, z0, u, parms):
    v = min(u[0], parms.v)
    omega = u[1]
    h = tspan[1] - tspan[0]
    x0, y0, theta0 = z0
    x1 = x0 + v * math.cos(theta0) * h
    y1 = y0 + v * math.sin(theta0) * h
    theta1 = theta0 + omega * h
    return [x1, y1, theta1]

# Smooths a given path using cubic B-spline
def smooth_path_with_spline(path, resolution=0.01):
    path = np.array(path)
    if path.ndim != 2 or path.shape[0] < 3:
        return path
    t = np.linspace(0, 1, len(path))
    spl_x = si.make_interp_spline(t, path[:, 0], k=3)
    spl_y = si.make_interp_spline(t, path[:, 1], k=3)
    t_new = np.arange(0, 1, resolution)
    x_smooth = spl_x(t_new)
    y_smooth = spl_y(t_new)
    return list(zip(x_smooth, y_smooth))


##TODO: 

# Runs Dynamic Window Approach for local path following
def dwa(x0, y0, theta0, v, goal, parms):
    omega_all = np.linspace(parms.omega_min, parms.omega_max, parms.n_omega)
    cost_all = np.zeros(parms.n_omega)
    trajectories = []

    for i, omega in enumerate(omega_all):
        z0 = [x0, y0, theta0]
        traj = [z0]
        for _ in range(parms.prediction_horizon):
            # use euler integration to get z0 , you can use the eluer_integration function
            #TODO
            z0 = ...
            traj.append(z0)
        traj = np.array(traj)
        trajectories.append(traj)

        valid = True
        for x, y, _ in traj:

            #update costs , use euclidian distances to the goal

            cost_all[i] += ...
            if not (parms.bounds[0] <= x <= parms.bounds[1] and parms.bounds[2] <= y <= parms.bounds[3]):
                cost_all[i] += 1e6
                valid = False
                break
            for ox, oy, r in parms.obstacles:
                if np.hypot(ox - x, oy - y) < r + parms.r_buffer + parms.R:
                    cost_all[i] += 1e6
                    valid = False
                    break
            if not valid:
                break

    best_index = np.argmin(cost_all)
    return omega_all[best_index], trajectories[best_index], trajectories



"""

END OF TBD 

"""

# ============================
# Main Execution
# ============================
parms = Parameters()
path_ = rrt_star(
    start=parms.start,
    goal=parms.goal,
    bounds=parms.bounds,
    obstacles=parms.obstacles,
    robot_radius=parms.robot_radius
)

path = smooth_path_with_spline(path_)
if path is None:
    raise ValueError("No RRT* path found.")

z0 = [path[0][0], path[0][1], 1.57]
z = [z0]
t = [0]
best_trajs = []
all_trajs = []
next_way_point = []

for goal in path[1:]:
    for _ in range(30):
        next_way_point.append(goal)
        x, y, theta = z0
        if np.hypot(goal[0] - x, goal[1] - y) < parms.r0:
            break
        omega, best_traj, all_trajectories = dwa(x, y, theta, parms.v, goal, parms)
        best_trajs.append(best_traj)
        all_trajs.append(all_trajectories)
        z0 = euler_integration([0, parms.dt], z0, [parms.v, omega], parms)
        z.append(z0)
        t.append(t[-1] + parms.dt)

z = np.array(z)
t = np.array(t)
t_interp = np.arange(t[0], t[-1], 1 / parms.fps)
z_interp = np.zeros((len(t_interp), z.shape[1]))
for i in range(z.shape[1]):
    f = interpolate.interp1d(t, z[:, i])
    z_interp[:, i] = f(t_interp)

# Animation
fig, ax = plt.subplots()
phi = np.arange(0, 2 * np.pi, 0.1)
plt.pause(1)
is_paused = True
for i in range(len(t_interp)):
    x, y, theta = z_interp[i]
    x_robot = x + parms.R * np.cos(phi)
    y_robot = y + parms.R * np.sin(phi)
    x2 = x + parms.R * np.cos(theta)
    y2 = y + parms.R * np.sin(theta)

    ax.clear()
    ax.plot([x, x2], [y, y2], 'k')
    ax.plot(x_robot, y_robot, 'k')
    ax.plot(z_interp[:i, 0], z_interp[:i, 1], 'b')
    ax.arrow(x, y, 0.2 * np.cos(theta), 0.2 * np.sin(theta),
             head_width=0.05, head_length=0.1, fc='black', ec='black')

    for ox, oy, r in parms.obstacles:
        ax.add_patch(plt.Circle((ox, oy), r, color='red'))

    xmin, xmax, ymin, ymax = parms.bounds
    ax.plot([xmin, xmax, xmax, xmin, xmin], [ymin, ymin, ymax, ymax, ymin], 'k-')

    if i < len(best_trajs):
        for traj in all_trajs[i]:
            ax.plot(traj[:, 0], traj[:, 1], 'gray', linestyle='-', linewidth=0.5)
        ax.plot(best_trajs[i][:, 0], best_trajs[i][:, 1], 'orange', linewidth=1.5)

    ax.plot(np.array(path)[:, 0], np.array(path)[:, 1], 'g--', label="Global Path")
    ax.add_patch(plt.Circle((parms.start[0], parms.start[1]), 0.05, color='black'))
    ax.add_patch(plt.Circle((parms.goal[0], parms.goal[1]), 0.05, color='red'))

    ax.set_xlim(xmin - 0.1, xmax + 0.1)
    ax.set_ylim(ymin - 0.1, ymax + 0.1)
    ax.set_aspect('equal')
    ax.set_title("DWA Following RRT* Global Path")
    plt.pause(parms.pause)
    while is_paused:
        plt.waitforbuttonpress()
        is_paused = False

plt.show()
