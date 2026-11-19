from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class PolicyConfig:
    """Parameters of the pick-and-place waypoint motion policy."""

    waypoints: tuple[tuple[float, float, float], ...] = (
        (0.45, -0.15, 0.35),
        (0.45, 0.15, 0.35),
        (0.45, 0.15, 0.18),
    )
    v_nom: float = 0.35
    a_max: float = 2.0
    k_approach: float = 4.0
    switch_tol: float = 0.01
    success_tol: float = 0.02


class MotionPolicy(ABC):
    """Base interface for TCP velocity policies."""

    @abstractmethod
    def act(self, tcp: np.ndarray, dt: float) -> np.ndarray:
        """Compute desired linear TCP velocity (m/s)."""

    @abstractmethod
    def is_done(self, tcp: np.ndarray) -> bool:
        """Return True when the task reaches its final target."""

    @abstractmethod
    def reset(self) -> None:
        """Reset internal policy state."""


class ScriptedWaypointPolicy(MotionPolicy):
    """Closed-loop TCP policy through hover, transfer, and place waypoints."""

    TRANSFER_INDEX = 1

    def __init__(self, cfg: PolicyConfig) -> None:
        if not cfg.waypoints:
            raise ValueError("waypoints must not be empty")
        self._cfg = cfg
        self._waypoints = [np.asarray(p, dtype=float) for p in cfg.waypoints]
        self._idx = 0
        self._v = np.zeros(3, dtype=float)

    def reset(self) -> None:
        self._idx = 0
        self._v = np.zeros(3, dtype=float)

    def act(self, tcp: np.ndarray, dt: float) -> np.ndarray:
        pos = np.asarray(tcp, dtype=float)
        err = self._waypoints[self._idx] - pos
        if np.linalg.norm(err) < self._cfg.switch_tol and self._idx < len(self._waypoints) - 1:
            self._idx += 1
            err = self._waypoints[self._idx] - pos
        dist = float(np.linalg.norm(err))
        speed = min(self._cfg.v_nom, self._cfg.k_approach * dist)
        goal = speed * err / dist if dist > 1e-9 else np.zeros(3, dtype=float)
        dv = goal - self._v
        dv_norm = float(np.linalg.norm(dv))
        max_dv = self._cfg.a_max * max(dt, 0.0)
        if dv_norm > max_dv > 0.0:
            dv *= max_dv / dv_norm
        self._v = self._v + dv
        return self._v.copy()

    def transfer_progress(self, tcp: np.ndarray) -> float:
        """Return normalized transfer progress in [0, 1], or NaN outside transfer."""
        if self._idx != self.TRANSFER_INDEX or len(self._waypoints) < 2:
            return math.nan
        a, b = self._waypoints[0], self._waypoints[1]
        ab = b - a
        denom = float(np.dot(ab, ab))
        return float(np.dot(np.asarray(tcp, dtype=float) - a, ab) / denom) if denom > 1e-12 else math.nan

    def is_done(self, tcp: np.ndarray) -> bool:
        return self._idx == len(self._waypoints) - 1 and (
            float(np.linalg.norm(self._waypoints[-1] - np.asarray(tcp, dtype=float))) < self._cfg.success_tol
        )
