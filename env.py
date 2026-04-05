from utils import find_pos, overlay_grid
from agent import Agent
from state_machine import StateMachine
from planner import dijkstra_risk_path, get_next_step

class DroneEnv:
    def __init__(self, config=None):
        self.config = config or {}
        self.map_data = None
        self.agent = None
        self.done = False
        self.step_count = 0

    def reset(self):
        self.done = False
        self.step_count = 0
        self.has_package = False
        self.delivered = False
        self.just_picked = False
        self.just_delivered = False
        self.just_charged = False
        self.just_hit_enemy = False

        self.grid = [
            "##########",
            "#...#....#",
            "#.#.#.##D#",
            "#.#E#..P.#",
            "#...C....#",
            "##########"
        ]

        self.agent_pos = (2, 1)

        self.package_pos = find_pos(self.grid, "P")
        self.delivery_pos = find_pos(self.grid, "D")
        self.charge_pos = find_pos(self.grid, "C")
        self.enemy_pos = find_pos(self.grid, "E")

        self.agent = Agent(self.agent_pos, energy=15)

        self.sm = StateMachine(
            grid=self.grid,
            package_pos=self.package_pos,
            delivery_pos=self.delivery_pos,
            charge_pos=self.charge_pos,
            enemy_pos=self.enemy_pos,

            # ===== 原有 =====
            safety_margin=2,
            enemy_danger_distance=1,
            enemy_safe_distance=2,
            avoid_steps=2,
            critical_energy_threshold=4,

            # ===== 新增 planner 参数 =====
            hard_block_distance=0,
            high_risk_distance=1,
            medium_risk_distance=2,
            high_risk_cost=8,
            medium_risk_cost=3,
        )

        overlay_grid(
            self.grid,
            {
                self.agent.pos: "A",
                self.enemy_pos: "E",
            }
        )

        obs = self._get_obs()
        return obs

    def step(self, action):
        if self.done:
            raise ValueError("Episode already done. Please call reset().")

        self.step_count += 1

        self.just_picked = False
        self.just_delivered = False
        self.just_charged = False
        self.just_hit_enemy = False

        self._apply_action(action)
        self._update_world()

        reward = self._get_reward()
        self.done = self._check_done()

        obs = self._get_obs()
        info = self._get_info()

        return obs, reward, self.done, info

    def _get_obs(self):
        return {
            "agent_pos": self.agent.pos,
            "energy": self.agent.energy,
            "has_package": self.has_package,
            "delivered": self.delivered,
            "package_pos": self.package_pos,
            "delivery_pos": self.delivery_pos,
            "charge_pos": self.charge_pos,
            "enemy_pos": self.enemy_pos,
            "step_count": self.step_count,
        }

    def _apply_action(self, action):
        r, c = self.agent.pos
        new_pos = (r, c)

        if action == 0:      # UP
            new_pos = (r - 1, c)
        elif action == 1:    # DOWN
            new_pos = (r + 1, c)
        elif action == 2:    # LEFT
            new_pos = (r, c - 1)
        elif action == 3:    # RIGHT
            new_pos = (r, c + 1)
        elif action == 4:    # STAY
            return
        elif action == 5:    # CHARGE
            if self.agent.pos == self.charge_pos:
                old_energy = self.agent.energy
                self.agent.energy = min(self.agent.energy + 5, 15)
                if self.agent.energy > old_energy:
                    self.just_charged = True
            return
        else:
            raise ValueError(f"Invalid action: {action}")

        if self._is_valid_pos(new_pos):
            self.agent.pos = new_pos
            self.agent.energy -= 1

    def _update_world(self):
        if self.agent.pos == self.enemy_pos:
            self.just_hit_enemy = True

        if (not self.has_package) and self.agent.pos == self.package_pos:
            self.has_package = True
            self.just_picked = True

        if self.has_package and self.agent.pos == self.delivery_pos and (not self.delivered):
            self.delivered = True
            self.just_delivered = True

    def _get_reward(self):
        if self.just_hit_enemy:
            return -20
        if self.just_delivered:
            return 20
        if self.agent.energy <= 0:
            return -10
        if self.just_picked:
            return 5
        return -1

    def _check_done(self):
        if self.just_hit_enemy:
            return True
        if self.delivered:
            return True
        if self.agent.energy <= 0:
            return True
        return False

    def _get_info(self):
        return {
            "has_package": self.has_package,
            "delivered": self.delivered,
            "just_picked": self.just_picked,
            "just_delivered": self.just_delivered,
            "just_hit_enemy": self.just_hit_enemy,
            "just_charged": self.just_charged,
        }
    
    def _is_valid_pos(self, pos):
        r, c = pos
        if r < 0 or r >= len(self.grid):
            return False
        if c < 0 or c >= len(self.grid[0]):
            return False
        return self.grid[r][c] != "#"
class RulePolicy:
    def act(self, obs):
        return 4