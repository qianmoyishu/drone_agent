from collections import deque
from utils import is_walkable


def get_neighbors(grid, pos):
    """
    返回当前位置上下左右四个可走邻居
    """
    x, y = pos
    directions = [
        (-1, 0),  # 上
        (1, 0),   # 下
        (0, -1),  # 左
        (0, 1),   # 右
    ]

    neighbors = []
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if is_walkable(grid, nx, ny):
            neighbors.append((nx, ny))
    return neighbors


def reconstruct_path(came_from, start, goal):
    """
    根据 came_from 反推完整路径
    """
    path = []
    current = goal

    while current != start:
        path.append(current)
        current = came_from[current]

    path.append(start)
    path.reverse()
    return path


def bfs(grid, start, goal):
    """
    使用 BFS 在网格地图中找从 start 到 goal 的最短路径
    找到返回路径列表，找不到返回 None
    """
    if start is None or goal is None:
        return None

    queue = deque([start])
    visited = set([start])
    came_from = {}

    while queue:
        current = queue.popleft()

        if current == goal:
            return reconstruct_path(came_from, start, goal)

        for neighbor in get_neighbors(grid, current):
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current
                queue.append(neighbor)

    return None