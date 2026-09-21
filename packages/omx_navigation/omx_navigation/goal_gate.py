"""Small, ROS-independent policy for when a navigation goal may run."""


class GoalGate:
    """A goal needs fresh localization and nobody else in control of the robot."""

    def __init__(self) -> None:
        self._ready = False
        self._goal_active = False
        self._inhibitors: set[str] = set()

    def accept_goal(self) -> bool:
        return self._ready and not self._inhibitors

    def refusal_reason(self) -> str:
        """Why `accept_goal` is false, in words an operator can act on."""
        if self._inhibitors:
            return ", ".join(sorted(self._inhibitors))
        if not self._ready:
            return "localization not ready"
        return ""

    @property
    def ready(self) -> bool:
        return self._ready

    @property
    def inhibitors(self) -> frozenset:
        return frozenset(self._inhibitors)

    def update_ready(self, ready: bool) -> bool:
        if not isinstance(ready, bool):
            raise ValueError("ready must be boolean")
        cancel = self._ready and not ready and self._goal_active
        self._ready = ready
        return cancel

    def update_inhibit(self, name: str, active: bool) -> bool:
        """Record one inhibitor; True when it has just started (act on it once)."""
        if not isinstance(active, bool):
            raise ValueError("active must be boolean")
        if not isinstance(name, str) or not name:
            raise ValueError("inhibitor name must be a nonempty string")
        started = active and name not in self._inhibitors
        if active:
            self._inhibitors.add(name)
        else:
            self._inhibitors.discard(name)
        return started

    def set_goal_active(self, active: bool) -> None:
        if not isinstance(active, bool):
            raise ValueError("active must be boolean")
        self._goal_active = active
