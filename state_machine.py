from enum import Enum
from planner import bfs


class AgentState(Enum):
    GO_PACKAGE = "GO_PACKAGE"
    GO_DELIVERY = "GO_DELIVERY"
    GO_CHARGE = "GO_CHARGE"


def path_steps(path):
    if path is None:
        return None
    return len(path) - 1


def need_charge_for_path(agent, path, safety_margin=2):
    steps = path_steps(path)
    if steps is None:
        return False
    return agent.energy < steps + safety_margin


def need_charge_for_task(agent, path_to_package, path_to_delivery, safety_margin=2):
    steps_to_package = path_steps(path_to_package)
    steps_to_delivery = path_steps(path_to_delivery)

    if steps_to_package is None or steps_to_delivery is None:
        return False

    total_required = steps_to_package + steps_to_delivery
    return agent.energy < total_required + safety_margin


class StateMachine:
    def __init__(self, grid, package_pos, delivery_pos, charge_pos, safety_margin=2):
        self.grid = grid
        self.package_pos = package_pos
        self.delivery_pos = delivery_pos
        self.charge_pos = charge_pos
        self.safety_margin = safety_margin

    def decide_state(self, agent):
        """
        根据当前 agent 状态决定下一目标状态
        """
        # 已经拿到包裹：优先考虑送货，否则去充电
        if agent.has_package:
            path_to_delivery = bfs(self.grid, agent.pos, self.delivery_pos)

            if need_charge_for_path(agent, path_to_delivery, self.safety_margin):
                return AgentState.GO_CHARGE
            return AgentState.GO_DELIVERY

        # 还没拿到包裹：判断能否完成完整任务
        path_to_package = bfs(self.grid, agent.pos, self.package_pos)
        path_package_to_delivery = bfs(self.grid, self.package_pos, self.delivery_pos)

        if need_charge_for_task(agent, path_to_package, path_package_to_delivery, self.safety_margin):
            return AgentState.GO_CHARGE
        return AgentState.GO_PACKAGE

    def get_target_by_state(self, state):
        """
        根据状态返回对应目标点
        """
        if state == AgentState.GO_PACKAGE:
            return self.package_pos
        if state == AgentState.GO_DELIVERY:
            return self.delivery_pos
        if state == AgentState.GO_CHARGE:
            return self.charge_pos
        return None

    def get_reason_by_state(self, state):
        """
        给当前状态一个可读说明，方便打印日志
        """
        if state == AgentState.GO_PACKAGE:
            return "前往包裹点"
        if state == AgentState.GO_DELIVERY:
            return "前往驿站送货"
        if state == AgentState.GO_CHARGE:
            return "当前电量不足，前往充电桩"
        return "未知状态"