from collections import deque
from utils import is_walkable


def bfs(grid, start, goal, blocked_positions=None):
    """
    BFS 最短路径搜索
    blocked_positions: 额外不可走的位置集合，比如敌人位置
    返回：
        路径坐标列表，例如 [(1,1), (1,2), (1,3)]
        如果不可达，返回 None
    """
    if blocked_positions is None:
        blocked_positions = set()
    else:
        blocked_positions = set(blocked_positions)

    if start in blocked_positions or goal in blocked_positions:
        return None

    queue = deque([start])
    visited = set([start])
    parent = {}

    while queue:
        current = queue.popleft()

        if current == goal:
            path = []
            while current in parent:
                path.append(current)
                current = parent[current]
            path.append(start)
            path.reverse()
            return path

        x, y = current
        neighbors = [
            (x - 1, y),
            (x + 1, y),
            (x, y - 1),
            (x, y + 1),
        ]

        for nx, ny in neighbors:
            nxt = (nx, ny)

            if nxt in visited:
                continue

            if nxt in blocked_positions:
                continue

            if not is_walkable(grid, nx, ny):
                continue

            visited.add(nxt)
            parent[nxt] = current
            queue.append(nxt)

    return None