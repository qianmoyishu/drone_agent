from env import DroneEnv,RulePolicy

def move_one_step(agent, path, title):
    """
    沿当前规划路径移动一步
    """
    if path is None:
        print(f"{title}：没有找到路径")
        return False

    if len(path) < 2:
        print(f"{title}：已经在目标点，无需移动")
        return True

    # 没电就不允许再移动
    if agent.energy <= 0:
        print(f"{title}：电量耗尽，无法继续移动")
        return False

    next_pos = path[1]
    agent.move_to(next_pos)

    print(
        f"{title} 单步移动 -> "
        f"pos={agent.pos} energy={agent.energy} "
        f"has_package={agent.has_package} score={agent.score}"
    )
    return True


def print_step_header(step_idx, grid, agent_pos, enemy_pos):
    print(f"========== Step {step_idx} ==========")
    overlay_grid(
        grid,
        {
            agent_pos: "A",
            enemy_pos: "E",
        }
    )
    print()


def print_agent_status(prefix, agent):
    print(
        f"{prefix}: pos={agent.pos}, energy={agent.energy}, "
        f"has_package={agent.has_package}, score={agent.score}"
    )


def handle_events(agent, package_pos, charge_pos, delivery_pos, current_state):
    if agent.pos == package_pos and not agent.has_package:
        agent.pickup_package()
        print("事件：成功拿到包裹")

    if agent.pos == charge_pos and current_state.value == "GO_CHARGE":
        agent.charge(full_energy=100)
        print("事件：充电完成")

    if agent.pos == delivery_pos and agent.has_package:
        agent.deliver_package()
        print("事件：包裹送达")
        print_agent_status("最终状态", agent)
        print("任务完成，程序结束")
        return "success"

    return "continue"


def check_terminal(agent, enemy_pos):
    """
    检查是否进入终止状态
    返回：
        "continue" / "fail"
    """
    if agent.pos == enemy_pos:
        print("事件：撞上敌人，任务失败")
        return "fail"

    if agent.energy < 0:
        print("事件：电量小于 0，任务失败")
        return "fail"

    # 宽松版规则：
    # energy == 0 允许停在原地，也允许刚好到达送货点/充电点
    # 但后续不能继续移动（move_one_step 已限制）
    return "continue"


# def main():
 
#     max_steps = 50

#     for step_idx in range(1, max_steps + 1):
#         print_step_header(step_idx, grid, agent.pos, enemy_pos)

#         current_state = sm.decide_state(agent)
#         reason = sm.get_reason_by_state(current_state)
#         target_pos = sm.get_target_by_state(agent, current_state)

#         print(f"当前状态: {current_state.value}")
#         print(f"状态说明: {reason}")
#         print(f"目标位置: {target_pos}")

#         if target_pos is None:
#             print("没有可用目标，程序结束")
#             return
        
#         enemy_positions = []
#         if enemy_pos is not None:
#             enemy_positions.append(enemy_pos)

#         path = dijkstra_risk_path(
#             grid=grid,
#             start=agent.pos,
#             goal=target_pos,
#             enemy_positions=enemy_positions,
#             extra_blocked_positions=None,   # 以后如果还有别的禁走点可以加这里
#             hard_block_distance=0,          # 敌人所在格绝对不能走
#             high_risk_distance=1,           # 敌人一圈高风险
#             medium_risk_distance=2,         # 再外一圈中风险
#             high_risk_cost=8,
#             medium_risk_cost=3,
#         )

#         print(f"当前规划路径: {path}")

#         ok = move_one_step(agent, path, current_state.value)
#         if not ok:
#             print("执行失败，程序结束")
#             return

#         sm.after_move(agent, current_state)

#         # 先处理碰撞 / 电量异常
#         terminal = check_terminal(agent, enemy_pos)
#         if terminal == "fail":
#             return

#         # 再处理拿包裹 / 充电 / 送达
#         event_result = handle_events(
#             agent,
#             package_pos,
#             charge_pos,
#             delivery_pos,
#             current_state,
#         )

#         if event_result == "success":
#             return

#         print_agent_status("回合结束状态", agent)
#         print()

#     print("达到最大步数，程序结束")
def main():
    env = DroneEnv()   # ✅ 必须有这一行
    policy = RulePolicy()

    obs = env.reset()
    print("reset:", obs)

    step = 0

    while step<10:
        action = policy.act(obs)
        obs, reward, done, info = env.step(action)

        print(f"step {step}, action={action}")
        print("obs:", obs)
        print("reward:", reward)
        print("done:", done)
        print("info:", info)
        print("------")

        step += 1

        if done:
            break

if __name__ == "__main__":
    main()