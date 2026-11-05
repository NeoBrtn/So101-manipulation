import numpy as np

from so101_manipulation.trajectory import GripperMonitor, TrajectoryStreamer, Waypoint


def run(streamer, q_meas_fn, t_end, dt=0.02):
    t, done = 0.0, False
    while t < t_end and not done:
        done = streamer.step(t, q_meas_fn(streamer), gripper_last_moving_t=-10.0)
        t += dt
    return t, done


def test_smoothstep_reaches_target_and_waits_to_settle():
    s = TrajectoryStreamer(joint_speed=1.0, min_duration=0.5)
    s.reset(np.zeros(5))
    s.start([Waypoint("a", np.full(5, 1.0), 0.0)])
    t, done = run(s, lambda st: st.q_cmd, 5.0)   # perfect tracking
    assert done and np.allclose(s.q_cmd, 1.0)
    assert 1.0 <= t < 1.1                        # 1 rad at 1 rad/s


def test_lagging_arm_times_out():
    s = TrajectoryStreamer(min_duration=0.5, settle_timeout=0.5)
    s.reset(np.zeros(5))
    s.start([Waypoint("a", np.full(5, 0.3), 0.0)])
    t, done = run(s, lambda st: np.zeros(5), 5.0)  # the arm never moves
    assert done and 1.0 <= t < 1.1


def test_cancel_holds_last_command():
    s = TrajectoryStreamer()
    s.reset(np.zeros(5))
    s.start([Waypoint("a", np.ones(5), 0.0, duration=1.0)])
    s.step(0.0, np.zeros(5))
    s.step(0.5, np.zeros(5))
    held = s.q_cmd.copy()
    s.cancel()
    assert not s.busy and 0.0 < held[0] < 1.0
    assert s.step(0.6, np.zeros(5)) and np.allclose(s.q_cmd, held)


def test_gripper_monitor():
    m = GripperMonitor(stall_speed=0.05)
    m.update(0.0, 0.0)
    m.update(0.1, 0.1)     # 1 rad/s: moving
    m.update(0.1, 0.2)     # stopped
    assert m.last_moving_t == 0.1
