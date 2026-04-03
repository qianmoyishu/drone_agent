def find_pos(grid, target):
    for i, row in enumerate(grid):
        for j, ch in enumerate(row):
            if ch == target:
                return (i, j)
    return None

def is_walkable(grid, x, y):
    if x < 0 or x >= len(grid):
        return False
    if y < 0 or y >= len(grid[0]):
        return False
    return grid[x][y] != '#'

def print_grid(grid):
    """
    打印地图
    """
    for row in grid:
        print(row)

def manhattan_distance(pos1, pos2):
    """
    曼哈顿距离
    """
    x1, y1 = pos1
    x2, y2 = pos2
    return abs(x1 - x2) + abs(y1 - y2)


def overlay_grid(grid, replacements):
    """
    在不修改原地图的情况下，打印带动态对象的位置
    replacements: dict {(x, y): char}
    """
    temp = [list(row) for row in grid]

    for (x, y), ch in replacements.items():
        if 0 <= x < len(temp) and 0 <= y < len(temp[0]):
            temp[x][y] = ch

    for row in temp:
        print("".join(row))
        