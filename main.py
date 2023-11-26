import heapq
import random


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = float('inf')
        self.h = float('inf')
        self.f = float('inf')
        self.parent = None

    def __lt__(self, other):
        return self.f < other.f



def heuristic(node, goal):
    return abs(node.x - goal.x) + abs(node.y - goal.y)


def get_neighbors(node, grid):
    neighbors = []
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 4-directional movement
        nx, ny = node.x + dx, node.y + dy
        if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]):
            neighbors.append(grid[nx][ny])
    return neighbors


def a_star(start, goal, grid):
    open_set = []
    heapq.heappush(open_set, start)
    start.g = 0
    start.h = heuristic(start, goal)
    start.f = start.g + start.h

    while open_set:
        current = heapq.heappop(open_set)
        if current == goal:
            return reconstruct_path(goal)

        for neighbor in get_neighbors(current, grid):
            temp_g = current.g + 1  # Assuming a grid where moving cost is always 1
            if temp_g < neighbor.g:
                neighbor.parent = current
                neighbor.g = temp_g
                neighbor.h = heuristic(neighbor, goal)
                neighbor.f = neighbor.g + neighbor.h
                if neighbor not in open_set:
                    heapq.heappush(open_set, neighbor)

    return None


def reconstruct_path(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]


def create_ascii_grid(grid, path, start, goal):
    ascii_grid = [["." for _ in range(len(grid[0]))] for _ in range(len(grid))]
    for x, y in path:
        ascii_grid[x][y] = "*"
    # Mark start and goal
    ascii_grid[start.x][start.y] = "S"
    ascii_grid[goal.x][goal.y] = "G"

    return "\n".join("".join(row) for row in ascii_grid)


# Initialize grid
grid_size = 299
grid = [[Node(i, j) for j in range(grid_size)] for i in range(grid_size)]
start = grid[0][0]

# Place the goal at a random spot
goal_x = random.randint(0, grid_size - 1)
goal_y = random.randint(0, grid_size - 1)
goal = grid[goal_x][goal_y]

# Find path using A* algorithm
path = a_star(start, goal, grid)

# Generate ASCII grid representation
ascii_grid = create_ascii_grid(grid, path, start, goal)
print(ascii_grid)
print(f"Goal Position: ({goal_x}, {goal_y})")
