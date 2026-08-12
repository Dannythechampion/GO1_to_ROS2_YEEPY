"""Small, ROS-independent readiness policy for navigation goals."""


class GoalGate:
    def __init__(self) -> None:
        self._ready = False
        self._goal_active = False

    def accept_goal(self) -> bool:
        return self._ready

    @property
    def ready(self) -> bool:
        return self._ready

    def update_ready(self, ready: bool) -> bool:
        if not isinstance(ready, bool):
            raise ValueError("ready must be boolean")
        cancel = self._ready and not ready and self._goal_active
        self._ready = ready
        if cancel:
            self._goal_active = False
        return cancel

    def set_goal_active(self, active: bool) -> None:
        if not isinstance(active, bool):
            raise ValueError("active must be boolean")
        self._goal_active = active
