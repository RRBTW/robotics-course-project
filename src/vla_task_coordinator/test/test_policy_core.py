import math
import numpy as np
import pytest

from vla_task_coordinator.policy_core import PolicyConfig, ScriptedWaypointPolicy


def test_policy_reaches_final_waypoint() -> None:
    cfg = PolicyConfig(
        waypoints=((0.4, -0.1, 0.3), (0.4, 0.1, 0.3), (0.4, 0.1, 0.2)),
        v_nom=0.4,
        a_max=4.0,
        k_approach=5.0,
        switch_tol=0.01,
        success_tol=0.02,
    )
    policy = ScriptedWaypointPolicy(cfg)
    tcp = np.array([0.35, -0.1, 0.3], dtype=float)
    dt = 0.05
    for _ in range(200):
        v = policy.act(tcp, dt)
        assert np.linalg.norm(v) <= cfg.v_nom + 1e-6
        tcp = tcp + v * dt
        if policy.is_done(tcp):
            break
    assert policy.is_done(tcp)


def test_acceleration_limit_and_reset() -> None:
    cfg = PolicyConfig(waypoints=((1.0, 0.0, 0.0),), v_nom=0.5, a_max=1.0)
    policy = ScriptedWaypointPolicy(cfg)
    v1 = policy.act(np.zeros(3), dt=0.05)
    assert np.linalg.norm(v1) == pytest.approx(0.05, abs=1e-6)
    policy.reset()
    assert math.isnan(policy.transfer_progress(np.zeros(3)))
