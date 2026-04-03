from utils import find_pos, print_grid
from planner import bfs
from agent import Agent
from state_machine import StateMachine


def move_one_step(agent, path, title):
    if path is None:
        print(f"{title}：没有找到路径")
        return False

    if len(path) < 2:
        print(f"{title}：已经在目标点，无需移动")
        return True

    next_pos = path[1]
    agent.move_to(next_pos)

    print(
        f"{title} 单步移动 -> pos={agent.pos} energy={agent.energy} "
        f"has_package={agent.has_package} score={agent.score}"
    )
    return True


def main():
    grid = [
        "##########",
        "#A..P...D#",
        "#..##....#",
        "#....C...#",
        "##########"
    ]

    print("当前地图：")
    print_grid(grid)
    print()

    agent_pos = find_pos(grid, 'A')
    package_pos = find_pos(grid, 'P')
    delivery_pos = find_pos(grid, 'D')
    charge_pos = find_pos(grid, 'C')

    agent = Agent(agent_pos, energy=8)

    sm = StateMachine(
        grid=grid,
        package_pos=package_pos,
        delivery_pos=delivery_pos,
        charge_pos=charge_pos,
        safety_margin=0
    )

    print(f"初始状态: pos={agent.pos}, energy={agent.energy}, has_package={agent.has_package}")
    print()

    max_steps = 30
    for step_idx in range(1, max_steps + 1):
        print(f"========== Step {step_idx} ==========")

        # 1. 决策状态
        current_state = sm.decide_state(agent)
        reason = sm.get_reason_by_state(current_state)
        target_pos = sm.get_target_by_state(current_state)

        print(f"当前状态: {current_state.value}")
        print(f"状态说明: {reason}")
        print(f"目标位置: {target_pos}")

        # 2. 规划路径
        path = bfs(grid, agent.pos, target_pos)
        print(f"当前规划路径: {path}")

        # 3. 只走一步
        ok = move_one_step(agent, path, current_state.value)
        if not ok:
            print("执行失败，程序结束")
            return

        # 4. 事件处理
        if agent.pos == package_pos and not agent.has_package:
            agent.pickup_package()
            print("事件：成功拿到包裹")

        if agent.pos == charge_pos:
            agent.charge(full_energy=10)
            print("事件：充电完成")

        if agent.pos == delivery_pos and agent.has_package:
            agent.deliver_package()
            print("事件：包裹送达")
            print(
                f"最终状态: pos={agent.pos}, energy={agent.energy}, "
                f"has_package={agent.has_package}, score={agent.score}"
            )
            print("任务完成，程序结束")
            break

        print(
            f"回合结束状态: pos={agent.pos}, energy={agent.energy}, "
            f"has_package={agent.has_package}, score={agent.score}"
        )
        print()


if __name__ == "__main__":
    main()