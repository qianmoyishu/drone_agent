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
