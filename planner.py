# planner.py
from collections import deque
import heapq


def get_neighbors(pos, grid):
    rows = len(grid)
    cols = len(grid[0])
    x, y = pos

    candidates = [
        (x - 1, y),  # up
        (x + 1, y),  # down
        (x, y - 1),  # left
        (x, y + 1),  # right
    ]

    valid = []
    for nx, ny in candidates:
        if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] != '#':
            valid.append((nx, ny))
    return valid


def reconstruct_path(parent, start, goal):
    if goal not in parent and goal != start:
        return []

    path = []
    cur = goal
    while cur != start:
        path.append(cur)
        cur = parent[cur]
    path.append(start)
    path.reverse()
    return path


def bfs_shortest_path(grid, start, goal, blocked_positions=None):
    """
    兼容你现在原来的 BFS 版本：
    - 避开障碍物 '#'
    - 避开 blocked_positions（比如 enemy_pos）
    """
    if start == goal:
        return [start]

    blocked_positions = set(blocked_positions or [])

    # goal 本身如果被 blocked，直接不可达
    if goal in blocked_positions:
        return []

    queue = deque([start])
    visited = {start}
    parent = {}

    while queue:
        cur = queue.popleft()

        for nxt in get_neighbors(cur, grid):
            if nxt in visited:
                continue
            if nxt in blocked_positions:
                continue

            visited.add(nxt)
            parent[nxt] = cur

            if nxt == goal:
                return reconstruct_path(parent, start, goal)

            queue.append(nxt)

    return []


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def build_risk_map(
    grid,
    enemy_positions,
    hard_block_distance=0,
    high_risk_distance=1,
    medium_risk_distance=2,
    high_risk_cost=8,
    medium_risk_cost=3,
):
    """
    构建风险代价图：
    - hard_block_distance 内：绝对禁走
    - high_risk_distance 内：高风险
    - medium_risk_distance 内：中风险

    返回：
    - blocked_positions: set[tuple]
    - risk_cost: dict[pos] = extra_cost
    """
    rows = len(grid)
    cols = len(grid[0])

    enemy_positions = [p for p in enemy_positions if p is not None]

    blocked_positions = set()
    risk_cost = {}

    if not enemy_positions:
        return blocked_positions, risk_cost

    for i in range(rows):
        for j in range(cols):
            pos = (i, j)

            if grid[i][j] == '#':
                continue

            min_dist = min(manhattan(pos, epos) for epos in enemy_positions)

            # 绝对禁走区
            if min_dist <= hard_block_distance:
                blocked_positions.add(pos)
                continue

            # 高风险区
            if min_dist <= high_risk_distance:
                risk_cost[pos] = max(risk_cost.get(pos, 0), high_risk_cost)

            # 中风险区
            elif min_dist <= medium_risk_distance:
                risk_cost[pos] = max(risk_cost.get(pos, 0), medium_risk_cost)

    return blocked_positions, risk_cost


def dijkstra_risk_path(
    grid,
    start,
    goal,
    enemy_positions=None,
    extra_blocked_positions=None,
    hard_block_distance=0,
    high_risk_distance=1,
    medium_risk_distance=2,
    high_risk_cost=8,
    medium_risk_cost=3,
):
    """
    风险感知路径规划：
    - 基础移动一步 cost = 1
    - 靠近敌人会附加风险 cost
    - 敌人所在格（默认 hard_block_distance=0）绝对禁走

    返回 path: [start, ..., goal]
    不可达返回 []
    """
    if start == goal:
        return [start]

    enemy_positions = enemy_positions or []
    extra_blocked_positions = set(extra_blocked_positions or [])

    enemy_blocked, risk_cost = build_risk_map(
        grid=grid,
        enemy_positions=enemy_positions,
        hard_block_distance=hard_block_distance,
        high_risk_distance=high_risk_distance,
        medium_risk_distance=medium_risk_distance,
        high_risk_cost=high_risk_cost,
        medium_risk_cost=medium_risk_cost,
    )

    blocked_positions = set(enemy_blocked) | extra_blocked_positions

    # 起点允许站着，不然有时你本来就贴着危险区会直接报死
    if start in blocked_positions:
        blocked_positions.remove(start)

    # 终点如果正好被绝对封死，直接不可达
    if goal in blocked_positions:
        return []

    pq = []
    heapq.heappush(pq, (0, start))

    dist = {start: 0}
    parent = {}

    while pq:
        cur_cost, cur = heapq.heappop(pq)

        if cur != start and cur_cost > dist.get(cur, float("inf")):
            continue

        if cur == goal:
            return reconstruct_path(parent, start, goal)

        for nxt in get_neighbors(cur, grid):
            if nxt in blocked_positions:
                continue

            move_cost = 1
            extra_cost = risk_cost.get(nxt, 0)
            new_cost = cur_cost + move_cost + extra_cost

            if new_cost < dist.get(nxt, float("inf")):
                dist[nxt] = new_cost
                parent[nxt] = cur
                heapq.heappush(pq, (new_cost, nxt))

    return []


def get_next_step(path):
    """
    给单步执行用：
    path 至少 [start, next] 才能走下一步
    """
    if not path or len(path) < 2:
        return None
    return path[1]