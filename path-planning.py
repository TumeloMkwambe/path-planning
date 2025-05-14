import heapq
import random
import numpy as np
from math import hypot
import matplotlib.pyplot as plt


# Tumelo Mkwambe - 2446873
# Salmaan Ebrahim - 1696622
# Amaan Hanslod - 2541305

#why we used PRM
#We used PRM because it works well for finding paths in large, complicated spaces with obstacles. Instead of checking every possible spot, it randomly samples safe points and connects them if the path is clear. It’s fast and works well when the map doesn’t change.
class Point:
    def __init__(self, r, c):
        self.r = r
        self.c = c


class Obstacle:
    def __init__(self, tr, tc, br, bc):
        self.tl = Point(min(tr, br), min(tc, bc))
        self.br = Point(max(tr, br), max(tc, bc))

    def map_obstacle(self, grid):
        for i in range(self.tl.r, self.br.r + 1):
            for j in range(self.tl.c, self.br.c + 1):
                if 0 <= i < grid.shape[0] and 0 <= j < grid.shape[1]:
                    grid[i][j] = -1


class Grid:
    def __init__(self, grid_rows, grid_columns):
        self.grid = np.zeros((grid_rows, grid_columns))
        self.start = None
        self.target = None
        self.obstacles = []

    def load_input(self, input_lines):
        point1, point2 = input_lines[0].split(';')
        r1, c1 = map(int, point1.split(','))
        r2, c2 = map(int, point2.split(','))
        self.start = Point(r1, c1)
        self.target = Point(r2, c2)

        for line in input_lines[1:]:
            if line.strip() == "-1":
                break
            point1, point2 = line.split(';')
            r1, c1 = map(int, point1.split(','))
            r2, c2 = map(int, point2.split(','))
            self.obstacles.append(Obstacle(r1, c1, r2, c2))
    
    def take_input(self):
        obstacles = []
        line = input()
        point1, point2 = line.split(';')
        r1, c1 = map(int, point1.split(','))
        r2, c2 = map(int, point2.split(','))
        self.start = Point(r1, c1)
        self.target = Point(r2, c2)
        while True:
            line = input()
            if line.strip() == "-1":
                break
            point1, point2 = line.split(';')
            r1, c1 = map(int, point1.split(','))
            r2, c2 = map(int, point2.split(','))
            obstacles.append(Obstacle(r1, c1, r2, c2))
        self.obstacles = np.array(obstacles)

    def map_obstacles(self):
        for obstacle in self.obstacles:
            obstacle.map_obstacle(self.grid)

    def mark_start_goal(self):
        self.grid[self.start.r][self.start.c] = 11
        self.grid[self.target.r][self.target.c] = 88


class PRMPlanner:
    def __init__(self, grid_obj, n_samples=200, k=10):
        self.grid = grid_obj.grid
        self.rows, self.cols = self.grid.shape
        self.start = (grid_obj.start.r, grid_obj.start.c)
        self.goal = (grid_obj.target.r, grid_obj.target.c)
        self.n_samples = n_samples
        self.k = k
        self.nodes = [self.start, self.goal]
        self.adj = {}

    def is_free(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols and self.grid[r, c] == 0

    def line_collision_free(self, p1, p2):
        (r1, c1), (r2, c2) = p1, p2
        steps = int(max(abs(r2 - r1), abs(c2 - c1)))
        if steps == 0:
            return self.is_free(r1, c1)
        for i in range(steps + 1):
            t = i / steps
            r = int(round(r1 + t * (r2 - r1)))
            c = int(round(c1 + t * (c2 - c1)))
            if not self.is_free(r, c):
                return False
        return True

    def sample_free(self):
        samples = set()
        attempts = 0
        while len(samples) < self.n_samples and attempts < self.n_samples * 10:
            r = random.randint(0, self.rows - 1)
            c = random.randint(0, self.cols - 1)
            if self.is_free(r, c):
                samples.add((r, c))
            attempts += 1
        return list(samples)

    def build_roadmap(self):
        sampled = self.sample_free()
        self.nodes.extend(sampled)

        for i in range(len(self.nodes)):
            self.adj[i] = []

        for i, p in enumerate(self.nodes):
            dists = []
            for j, q in enumerate(self.nodes):
                if i == j:
                    continue
                d = hypot(p[0] - q[0], p[1] - q[1])
                dists.append((d, j))
            dists.sort()
            for _, j in dists[:self.k]:
                if self.line_collision_free(p, self.nodes[j]):
                    cost = hypot(p[0] - self.nodes[j][0], p[1] - self.nodes[j][1])
                    self.adj[i].append((j, cost))
                    self.adj[j].append((i, cost))  # undirected

    def dijkstra(self):
        N = len(self.nodes)
        dist = [float('inf')] * N
        prev = [None] * N
        dist[0] = 0
        pq = [(0, 0)]
        visited = [False] * N

        while pq:
            d, u = heapq.heappop(pq)
            if visited[u]:
                continue
            visited[u] = True
            if u == 1:
                break
            for v, w in self.adj[u]:
                if dist[u] + w < dist[v]:
                    dist[v] = dist[u] + w
                    prev[v] = u
                    heapq.heappush(pq, (dist[v], v))

        if not visited[1]:
            return None

        path = []
        cur = 1
        while cur is not None:
            path.append(self.nodes[cur])
            cur = prev[cur]
        return path[::-1]

    def plan(self):
        self.build_roadmap()
        return self.dijkstra()


def plot_path(grid, path):
    plt.figure(figsize=(10, 10))
    rows, cols = grid.shape

    #show gridfree space is 0, obstacles are -1
    plt.imshow(grid == -1, cmap='gray', origin='lower')

    # pplot the path if found
    if path:
        path_r, path_c = zip(*path)
        plt.plot(path_c, path_r, color='blue', linewidth=2, label='Path')

        #mark start and goal
        plt.scatter(path_c[0], path_r[0], color='green', s=100, label='Start')
        plt.scatter(path_c[-1], path_r[-1], color='red', s=100, label='Goal')

    plt.title("PRM Path Planning")
    plt.legend()
    plt.grid(True)
    plt.show()

#simulation ====

g = Grid(100, 100)
g.take_input()
g.map_obstacles()
g.mark_start_goal()
g.grid[g.start.r, g.start.c] = 0  #markers for PRM
g.grid[g.target.r, g.target.c] = 0

planner = PRMPlanner(g, n_samples=300, k=10)
path = planner.plan()

if path:
    for r, c in path:
        print(f"{r},{c}")
else:
    print("No path found.")

#plot
plot_path(g.grid, path)


