from enum import Enum
from planner import dijkstra_risk_path
from utils import manhattan_distance, is_walkable


class AgentState(Enum):
    GO_PACKAGE = "GO_PACKAGE"
    GO_DELIVERY = "GO_DELIVERY"
    GO_CHARGE = "GO_CHARGE"
    AVOID_ENEMY = "AVOID_ENEMY"


def path_steps(path):
    if path is None or len(path) == 0:
        return None
    return len(path) - 1


class StateMachine:
    def __init__(
        self,
        grid,
        package_pos,
        delivery_pos,
        charge_pos,
        enemy_pos=None,

        # ===== 原有参数 =====
        safety_margin=2,
        enemy_danger_distance=2,
        enemy_safe_distance=3,
        avoid_steps=3,
        critical_energy_threshold=2,

        # ===== 新增：planner 参数 =====
        hard_block_distance=0,
        high_risk_distance=1,
        medium_risk_distance=2,
        high_risk_cost=8,
        medium_risk_cost=3,
    ):
        self.grid = grid
        self.package_pos = package_pos
        self.delivery_pos = delivery_pos
        self.charge_pos = charge_pos
        self.enemy_pos = enemy_pos

        # ===== 原有 =====
        self.safety_margin = safety_margin
        self.enemy_danger_distance = enemy_danger_distance
        self.enemy_safe_distance = enemy_safe_distance
        self.avoid_steps = avoid_steps
        self.critical_energy_threshold = critical_energy_threshold

        # ===== 新增：planner 参数 =====
        self.hard_block_distance = hard_block_distance
        self.high_risk_distance = high_risk_distance
        self.medium_risk_distance = medium_risk_distance
        self.high_risk_cost = high_risk_cost
        self.medium_risk_cost = medium_risk_cost

        # ===== 避障状态 =====
        self.in_avoid_mode = False
        self.avoid_target = None

    # =========================
    # 基础工具
    # =========================
    def get_enemy_distance(self, pos):
        if self.enemy_pos is None:
            return 999999
        return manhattan_distance(pos, self.enemy_pos)

    def is_enemy_near(self, agent):
        return self.get_enemy_distance(agent.pos) <= self.enemy_danger_distance

    def is_enemy_far_enough(self, agent):
        return self.get_enemy_distance(agent.pos) >= self.enemy_safe_distance

    def is_position_safe(self, pos):
        return self.get_enemy_distance(pos) >= self.enemy_safe_distance

    def get_neighbors(self, pos):
        x, y = pos
        candidates = [
            (x - 1, y),
            (x + 1, y),
            (x, y - 1),
            (x, y + 1),
        ]
        return [(nx, ny) for nx, ny in candidates if is_walkable(self.grid, nx, ny)]

    def can_reach_with_margin(self, agent, path, safety_margin=None):
        if safety_margin is None:
            safety_margin = self.safety_margin

        steps = path_steps(path)
        if steps is None:
            return False

        return agent.energy >= steps + safety_margin

    def can_reach_without_margin(self, agent, path):
        steps = path_steps(path)
        if steps is None:
            return False
        return agent.energy >= steps

    def get_main_target(self, agent):
        if agent.has_package:
            return self.delivery_pos
        return self.package_pos

    def plan_path(self, start, goal):
        enemy_positions = []
        if self.enemy_pos is not None:
            enemy_positions.append(self.enemy_pos)

        return dijkstra_risk_path(
            grid=self.grid,
            start=start,
            goal=goal,
            enemy_positions=enemy_positions,
            hard_block_distance=0,
            high_risk_distance=1,
            medium_risk_distance=2,
            high_risk_cost=8,
            medium_risk_cost=3,
        )

    # =========================
    # 电量与任务判断
    # =========================
    def can_finish_direct_delivery(self, agent):
        """
        未拿包裹时，是否能直接完成：
        当前 -> 包裹 -> 驿站
        """
        path_to_package = self.plan_path(agent.pos, self.package_pos)
        path_package_to_delivery = self.plan_path(self.package_pos, self.delivery_pos)

        s1 = path_steps(path_to_package)
        s2 = path_steps(path_package_to_delivery)

        if s1 is None or s2 is None:
            return False

        total_steps = s1 + s2
        return agent.energy >= total_steps + self.safety_margin

    def is_critical_charge_needed(self, agent):
        """
        危急充电：
        1. 已拿包裹
        2. 电量低
        3. 至少还能走到充电桩
        """
        if not agent.has_package:
            return False

        if agent.energy > self.critical_energy_threshold:
            return False

        path_to_charge = self.plan_path(agent.pos, self.charge_pos)
        return self.can_reach_without_margin(agent, path_to_charge)

    def choose_pre_package_strategy(self, agent):
        """
        未拿包裹时比较：
        1. 先充电：当前位置 -> 充电桩 -> 包裹 -> 驿站
        2. 先拿包裹再充电：当前位置 -> 包裹 -> 充电桩 -> 驿站
        """
        # 方案1：先充电
        path_to_charge = self.plan_path(agent.pos, self.charge_pos)
        path_charge_to_package = self.plan_path(self.charge_pos, self.package_pos)
        path_package_to_delivery = self.plan_path(self.package_pos, self.delivery_pos)

        a = path_steps(path_to_charge)
        b = path_steps(path_charge_to_package)
        c = path_steps(path_package_to_delivery)

        if a is None or b is None or c is None:
            total_charge_first = None
        else:
            total_charge_first = a + b + c

        # 方案2：先拿包裹再充电
        path_to_package = self.plan_path(agent.pos, self.package_pos)
        path_package_to_charge = self.plan_path(self.package_pos, self.charge_pos)
        path_charge_to_delivery = self.plan_path(self.charge_pos, self.delivery_pos)

        can_package_first = self.can_reach_with_margin(
            agent, path_to_package, self.safety_margin
        )

        d = path_steps(path_to_package)
        e = path_steps(path_package_to_charge)
        f = path_steps(path_charge_to_delivery)

        if can_package_first and d is not None and e is not None and f is not None:
            total_package_first = d + e + f
        else:
            total_package_first = None

        if total_charge_first is None and total_package_first is None:
            return "charge_first"
        if total_charge_first is None:
            return "package_first"
        if total_package_first is None:
            return "charge_first"

        if total_package_first < total_charge_first:
            return "package_first"
        return "charge_first"

    # =========================
    # 避障逻辑
    # =========================
    def find_safe_positions(self, agent):
        safe_positions = []
        rows = len(self.grid)
        cols = len(self.grid[0])

        for x in range(rows):
            for y in range(cols):
                if not is_walkable(self.grid, x, y):
                    continue
                if self.is_position_safe((x, y)):
                    safe_positions.append((x, y))

        return safe_positions

    def should_keep_current_avoid_target(self, agent):
        """
        当前已有避障目标时，尽量继续沿用，避免每步重选导致抖动。
        """
        if self.avoid_target is None:
            return False

        if self.avoid_target == agent.pos:
            return False

        if not self.is_position_safe(self.avoid_target):
            return False

        path = self.plan_path(agent.pos, self.avoid_target)
        if path is None or len(path) < 2:
            return False

        return True

    def choose_avoid_target(self, agent):
        """
        进入避障时选一个临时安全点：
        1. 必须安全
        2. 必须可达
        3. 尽量离当前近
        4. 但也别太偏离主任务
        """
        safe_positions = self.find_safe_positions(agent)
        if not safe_positions:
            return agent.pos

        main_target = self.get_main_target(agent)
        candidates = []

        for pos in safe_positions:
            if pos == agent.pos:
                continue

            path = self.plan_path(agent.pos, pos)
            steps = path_steps(path)
            if steps is None:
                continue

            enemy_dist = self.get_enemy_distance(pos)
            target_dist = manhattan_distance(pos, main_target)

            # 排序逻辑：
            # 先选更近的安全点
            # 同距离下更远离敌人
            # 再同分时更接近主任务目标
            candidates.append((steps, -enemy_dist, target_dist, pos))

        if not candidates:
            return agent.pos

        candidates.sort()
        return candidates[0][3]

    def get_fallback_avoid_step(self, agent):
        """
        如果安全点策略失效，就退化到邻居级避让。
        更强地惩罚回头走，减少横跳。
        """
        neighbors = self.get_neighbors(agent.pos)
        if not neighbors:
            return agent.pos

        scored = []
        for pos in neighbors:
            enemy_dist = self.get_enemy_distance(pos)

            # 强惩罚回到上一步
            backtrack_penalty = 2 if agent.prev_pos is not None and pos == agent.prev_pos else 0

            # 轻微偏好“更接近主任务”的方向，避免一味乱跑
            main_target = self.get_main_target(agent)
            target_dist = manhattan_distance(pos, main_target)

            # 排序：
            # 1. 尽量不回头
            # 2. 尽量离敌人远
            # 3. 尽量别偏离主任务太远
            scored.append((backtrack_penalty, -enemy_dist, target_dist, pos))

        scored.sort()
        return scored[0][3]

    def get_avoid_target(self, agent):
        """
        避障时优先沿用已有安全点。
        只有当前目标失效时才重选。
        """
        if self.should_keep_current_avoid_target(agent):
            return self.avoid_target

        self.avoid_target = self.choose_avoid_target(agent)

        if self.avoid_target == agent.pos:
            return self.get_fallback_avoid_step(agent)

        path = self.plan_path(agent.pos, self.avoid_target)
        if path is None or len(path) < 2:
            return self.get_fallback_avoid_step(agent)

        return self.avoid_target

    def enter_avoid_mode(self, agent):
        self.in_avoid_mode = True
        agent.avoid_cooldown = self.avoid_steps

        if not self.should_keep_current_avoid_target(agent):
            self.avoid_target = self.choose_avoid_target(agent)

    def exit_avoid_mode(self, agent):
        self.in_avoid_mode = False
        agent.avoid_cooldown = 0
        self.avoid_target = None

    # =========================
    # 状态决策
    # =========================
    def should_force_delivery(self, agent):
        """
        已拿包裹且离终点很近时，允许压过普通避障，直接冲刺送达
        """
        if not agent.has_package:
            return False

        path_to_delivery = self.plan_path(agent.pos, self.delivery_pos)
        steps = path_steps(path_to_delivery)

        if steps is None:
            return False

        # 至少得走得到
        if not self.can_reach_without_margin(agent, path_to_delivery):
            return False

        # 剩余 2 步内直接冲刺
        return steps <= 2

    def decide_state(self, agent):
        """
        优先级：
        1. 危急充电
        2. 终点冲刺
        3. 维持避障模式
        4. 新进入避障
        5. 正常任务逻辑
        """

        # P1：危急充电优先
        if self.is_critical_charge_needed(agent):
            self.exit_avoid_mode(agent)
            return AgentState.GO_CHARGE

        # P2：终点冲刺优先
        if self.should_force_delivery(agent):
            self.exit_avoid_mode(agent)
            return AgentState.GO_DELIVERY

        # P3：已经在避障模式中
        if self.in_avoid_mode:
            # 真正满足“已足够安全 + 冷却结束”才退出
            if self.is_enemy_far_enough(agent) and agent.avoid_cooldown <= 0:
                self.exit_avoid_mode(agent)
            else:
                return AgentState.AVOID_ENEMY

        # P4：不在避障模式，但敌人过近
        if self.enemy_pos is not None and self.is_enemy_near(agent):
            self.enter_avoid_mode(agent)
            return AgentState.AVOID_ENEMY

        # P5：正常任务逻辑
        if agent.has_package:
            path_to_delivery = self.plan_path(agent.pos, self.delivery_pos)

            if self.can_reach_with_margin(agent, path_to_delivery, self.safety_margin):
                return AgentState.GO_DELIVERY

            return AgentState.GO_CHARGE

        # P6：未拿包裹时，先看能否直接完成配送
        if self.can_finish_direct_delivery(agent):
            return AgentState.GO_PACKAGE

        # P7：否则比较先充电 or 先拿包裹再充电
        strategy = self.choose_pre_package_strategy(agent)
        if strategy == "package_first":
            return AgentState.GO_PACKAGE
        return AgentState.GO_CHARGE

    # =========================
    # 状态 -> 目标
    # =========================
    def get_target_by_state(self, agent, state):
        if state == AgentState.GO_PACKAGE:
            return self.package_pos
        if state == AgentState.GO_DELIVERY:
            return self.delivery_pos
        if state == AgentState.GO_CHARGE:
            return self.charge_pos
        if state == AgentState.AVOID_ENEMY:
            return self.get_avoid_target(agent)
        return None

    def get_reason_by_state(self, state):
        if state == AgentState.GO_PACKAGE:
            return "前往包裹点"
        if state == AgentState.GO_DELIVERY:
            return "前往驿站送货"
        if state == AgentState.GO_CHARGE:
            return "当前电量不足，前往充电桩"
        if state == AgentState.AVOID_ENEMY:
            return "敌人过近，先撤到安全位置"
        return "未知状态"

    def after_move(self, agent, state):
        """
        每步之后更新内部状态
        """
        if state == AgentState.AVOID_ENEMY and agent.avoid_cooldown > 0:
            agent.avoid_cooldown -= 1

        # 到了安全目标点后，如果也满足退出条件，就退出避障
        if state == AgentState.AVOID_ENEMY:
            if self.avoid_target is not None and agent.pos == self.avoid_target:
                if self.is_enemy_far_enough(agent) and agent.avoid_cooldown <= 0:
                    self.exit_avoid_mode(agent)