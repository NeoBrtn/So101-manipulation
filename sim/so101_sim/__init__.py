# SO101Sim — MuJoCo simulation backend of the SO-ARM101.
#
# Exposes STRICTLY the same interface as lerobot.robots.so_follower.SO101Follower:
#   connect() / disconnect() / get_observation() / send_action() / is_connected
#
# The ROS2 driver node must work the same way
# with SO101Sim or SO101Follower (ROS2 parameter `use_sim`).
#
# Units: radians for the 5 arm joints, 0-100 for the gripper
# (same as SO101FollowerConfig(use_radians=True) on the real arm).

import time
from pathlib import Path

import mujoco
import numpy as np

ASSETS_DIR = Path("/opt/so101/sim/so101_sim/assets/so101")
DEFAULT_SCENE = ASSETS_DIR / "scene_tek5.xml"

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER = "gripper"
ALL_JOINTS = ARM_JOINTS + [GRIPPER]

# Gripper opening: mapping 0-100 (LeRobot convention) <-> MJCF radians
GRIPPER_RANGE_RAD = None  # filled at init from the model


class SO101Sim:
    """Digital twin of the SO-ARM101. Same API as SO101Follower."""

    def __init__(
        self,
        scene_path: str | Path = DEFAULT_SCENE,
        camera_name: str = "external_cam",
        camera_width: int = 320,
        camera_height: int = 240,
        sim_dt_per_step: int = 5,
        seed: int | None = None,
    ):
        self.scene_path = str(scene_path)
        self.camera_name = camera_name
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.sim_dt_per_step = sim_dt_per_step
        self._rng = np.random.default_rng(seed)

        self.model: mujoco.MjModel | None = None
        self.data: mujoco.MjData | None = None
        self._renderer: mujoco.Renderer | None = None
        self._connected = False
        self._last_step_t = 0.0

    # ------------------------------------------------------------------ API

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self, calibrate: bool = True) -> None:  # same signature as the real arm
        self.model = mujoco.MjModel.from_xml_path(self.scene_path)
        self.data = mujoco.MjData(self.model)

        self._joint_qpos_adr = {
            name: self.model.joint(name).qposadr[0] for name in ALL_JOINTS
        }
        self._actuator_id = {
            name: self.model.actuator(name).id for name in ALL_JOINTS
        }
        g_jnt = self.model.joint(GRIPPER)
        self._gripper_range = (float(g_jnt.range[0]), float(g_jnt.range[1]))

        self._setup_screws()
        self._setup_grip_assist()
        mujoco.mj_forward(self.model, self.data)
        self.randomize_object()
        self._connected = True
        self._last_step_t = time.perf_counter()

    def disconnect(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        self._connected = False

    def get_observation(self) -> dict:
        """Same keys as the real arm ('<joint>.pos').
        The camera image is NOT included; callers must call
        render_camera() explicitly when they need it.
        """
        self._require_connected()
        self._step_sim()
        obs = {}
        for name in ARM_JOINTS:
            obs[f"{name}.pos"] = float(self.data.qpos[self._joint_qpos_adr[name]])
        obs[f"{GRIPPER}.pos"] = self._gripper_rad_to_pct(
            self.data.qpos[self._joint_qpos_adr[GRIPPER]]
        )
        return obs

    def send_action(self, action: dict) -> dict:
        """Position targets. Expected keys: '<joint>.pos' (radians, gripper 0-100)."""
        self._require_connected()
        applied = {}
        for key, val in action.items():
            name = key.removesuffix(".pos")
            if name not in self._actuator_id:
                continue
            if name == GRIPPER:
                target = self._gripper_pct_to_rad(val)
            else:
                target = val
            self.data.ctrl[self._actuator_id[name]] = target
            applied[key] = val
        self._step_sim()
        return applied

    # ------------------------------------------------------- sim utilities

    def render_camera(self) -> np.ndarray:
        """RGB image (H, W, 3) uint8 of the fixed external camera."""
        if self._renderer is None:
            self._renderer = mujoco.Renderer(
                self.model, height=self.camera_height, width=self.camera_width
            )
        self._renderer.update_scene(self.data, camera=self.camera_name)
        return self._renderer.render()

    def randomize_object(self, radius_range=(0.18, 0.30), angle_range_deg=(-50, 60)) -> None:
        """Moves pickup_object to a reachable position, with a random yaw. Scenes without
        a pickup_object reset their objects (and screw caps) to the initial state."""
        if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "pickup_object_free") < 0:
            self.reset_objects()
            return
        r = self._rng.uniform(*radius_range)
        a = np.radians(self._rng.uniform(*angle_range_deg))
        yaw = self._rng.uniform(0, np.pi / 2)  # the cube is invariant under 90° rotation
        adr = self.model.joint("pickup_object_free").qposadr[0]
        self.data.qpos[adr : adr + 3] = [r * np.cos(a), r * np.sin(a), 0.03]
        self.data.qpos[adr + 3 : adr + 7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        self.data.qvel[:] = 0
        # let the object settle on the ground (physics settling)
        for _ in range(200):
            mujoco.mj_step(self.model, self.data)

    def get_object_position(self) -> np.ndarray:
        """World position of the object — DEBUG/EVALUATION only.
        Forbidden in the pipeline: the position must come from perception."""
        return self.data.body("pickup_object").xpos.copy()

    def get_drop_box_position(self) -> np.ndarray:
        """World position of the drop_box."""
        return self.data.body("drop_box").xpos.copy()

    def get_camera_extrinsics(self) -> np.ndarray:
        """Camera->world pose (4x4). This is the 'provided TF' of the subject."""
        cam = self.data.camera(self.camera_name)
        T = np.eye(4)
        T[:3, :3] = cam.xmat.reshape(3, 3)
        T[:3, 3] = cam.xpos
        return T

    def get_camera_intrinsics(self) -> np.ndarray:
        """K matrix (3x3) derived from the MJCF fovy and the render resolution."""
        cam_id = self.model.camera(self.camera_name).id
        fovy = np.radians(self.model.cam_fovy[cam_id])
        fy = self.camera_height / (2 * np.tan(fovy / 2))
        fx = fy  # square pixels
        return np.array(
            [
                [fx, 0, self.camera_width / 2],
                [0, fy, self.camera_height / 2],
                [0, 0, 1],
            ]
        )

    def reset_objects(self) -> None:
        """Everything but the arm back to the scene's initial state; screw caps screwed."""
        arm = {self._joint_qpos_adr[n] for n in ALL_JOINTS}
        for j in range(self.model.njnt):
            if self.model.jnt_qposadr[j] in arm:
                continue
            q, v = self.model.jnt_qposadr[j], self.model.jnt_dofadr[j]
            kind = int(self.model.jnt_type[j])   # mjtJoint is not an int: compare values
            nq = {int(mujoco.mjtJoint.mjJNT_FREE): 7, int(mujoco.mjtJoint.mjJNT_BALL): 4}.get(kind, 1)
            nv = {int(mujoco.mjtJoint.mjJNT_FREE): 6, int(mujoco.mjtJoint.mjJNT_BALL): 3}.get(kind, 1)
            self.data.qpos[q:q + nq] = self.model.qpos0[q:q + nq]
            self.data.qvel[v:v + nv] = 0
        for screw in self._screws:
            self._engage_screw(screw)
        for grip in self._grips:
            self.data.eq_active[grip["eq"]] = 0
        mujoco.mj_forward(self.model, self.data)
        for _ in range(200):   # settle
            mujoco.mj_step(self.model, self.data)

    def get_screw_states(self) -> dict:
        """{cap: (turns done, "engaged" | "loose" | "free")} — DEBUG/EVALUATION only."""
        return {s["name"]: (float(self.data.qpos[s["turn_qpos"]]) / (2 * np.pi), s["state"])
                for s in self._screws}

    # -------------------------------------------------------------- private

    def _step_sim(self) -> None:
        for _ in range(self.sim_dt_per_step):
            mujoco.mj_step(self.model, self.data)
            if self._screws:
                self._update_screws()
            if self._grips:
                self._update_grip_assist()

    # Screw caps. A scene declares one with <numeric name="screw:<cap>" data="<turns>"/> and:
    #   body <cap> with a hinge <cap>_turn and a slide <cap>_lift (relative to the bottle),
    #   coupled by the equality <cap>_thread (lift = pitch / 2pi * turn),
    #   body <cap>_free with a freejoint: the same cap once off the bottle.
    # States: engaged (thread) -> loose after <turns> turns (thread off: it spins and slides
    # up along the neck freely but stays on it, like a real hollow cap) -> free once lifted
    # LIFT_OFF above where it came loose: swapped for <cap>_free at the same pose.

    LIFT_OFF = 0.006         # m
    LOOSE_FRICTION = 0.002   # N m once the thread is out

    def _setup_screws(self) -> None:
        m = self.model
        self._screws = []
        for i in range(m.nnumeric):
            name = m.numeric(i).name
            if not name.startswith("screw:"):
                continue
            cap = name.removeprefix("screw:")
            free = m.body(f"{cap}_free")
            screw = {
                "name": cap,
                "turns": float(m.numeric(i).data[0]),
                "body": m.body(cap).id,
                "turn_qpos": m.joint(f"{cap}_turn").qposadr[0],
                "eq": m.equality(f"{cap}_thread").id,
                "free_qpos": m.joint(f"{cap}_free").qposadr[0],
                "free_dof": m.joint(f"{cap}_free").dofadr[0],
                "free_body": free.id,
                "lift_qpos": m.joint(f"{cap}_lift").qposadr[0],
                "lift_jnt": m.joint(f"{cap}_lift").id,
                "turn_jnt": m.joint(f"{cap}_turn").id,
                "turn_dof": m.joint(f"{cap}_turn").dofadr[0],
                "lift_range": m.jnt_range[m.joint(f"{cap}_lift").id].copy(),
                "turn_range": m.jnt_range[m.joint(f"{cap}_turn").id].copy(),
                "friction": float(m.dof_frictionloss[m.joint(f"{cap}_turn").dofadr[0]]),
                "geoms": [g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body(cap).id],
                "free_geoms": [g for g in range(m.ngeom) if m.geom_bodyid[g] == free.id],
                "state": "engaged",
            }
            screw["collision"] = {g: (int(m.geom_contype[g]), int(m.geom_conaffinity[g]))
                                  for g in screw["geoms"] + screw["free_geoms"]}
            self._screws.append(screw)
            self._engage_screw(screw)

    def _set_present(self, screw, geoms, present: bool) -> None:
        for g in geoms:
            contype, conaffinity = screw["collision"][g]
            self.model.geom_contype[g] = contype if present else 0
            self.model.geom_conaffinity[g] = conaffinity if present else 0
            self.model.geom_rgba[g, 3] = 1.0 if present else 0.0

    def _engage_screw(self, screw) -> None:
        m = self.model
        self._set_present(screw, screw["geoms"], True)
        self._set_present(screw, screw["free_geoms"], False)
        m.body_gravcomp[screw["free_body"]] = 1.0   # parked: does not fall
        m.jnt_range[screw["lift_jnt"]] = screw["lift_range"]
        m.jnt_range[screw["turn_jnt"]] = screw["turn_range"]
        m.dof_frictionloss[screw["turn_dof"]] = screw["friction"]
        self.data.eq_active[screw["eq"]] = 1
        screw["state"] = "engaged"

    def _update_screws(self) -> None:
        m, d = self.model, self.data
        for screw in self._screws:
            if screw["state"] == "engaged":
                if d.qpos[screw["turn_qpos"]] < screw["turns"] * 2 * np.pi:
                    continue
                # thread out: spins freely, rests on the thread end, can be lifted off
                lift = float(d.qpos[screw["lift_qpos"]])
                d.eq_active[screw["eq"]] = 0
                m.jnt_range[screw["lift_jnt"]] = [lift, lift + 2 * self.LIFT_OFF]
                m.jnt_range[screw["turn_jnt"]] = [-1e3, 1e3]
                m.dof_frictionloss[screw["turn_dof"]] = self.LOOSE_FRICTION
                screw["loose_lift"] = lift
                screw["state"] = "loose"
                continue
            if screw["state"] != "loose" or d.qpos[screw["lift_qpos"]] < screw["loose_lift"] + self.LIFT_OFF:
                continue
            q, v = screw["free_qpos"], screw["free_dof"]
            self.data.qpos[q:q + 3] = self.data.xpos[screw["body"]]
            self.data.qpos[q + 3:q + 7] = self.data.xquat[screw["body"]]
            self.data.qvel[v:v + 6] = 0
            self._set_present(screw, screw["geoms"], False)
            self._set_present(screw, screw["free_geoms"], True)
            self.model.body_gravcomp[screw["free_body"]] = 0.0
            screw["state"] = "free"
            mujoco.mj_forward(self.model, self.data)   # poses of the free cap, before welding it
            # held by the grip assist: the free cap stays in the gripper
            for grip in self._grips:
                if grip["body"] == screw["body"] and self.data.eq_active[grip["eq"]]:
                    self.data.eq_active[grip["eq"]] = 0
                    for other in self._grips:
                        if other["body"] == screw["free_body"]:
                            self._weld(other)
            mujoco.mj_forward(self.model, self.data)

    # Grip assist (stands in for grip pads, simulation only). A scene declares it with
    # <numeric name="grip_assist:<body>" data="1"/> and a weld <equality> named
    # grip_<body> between the moving jaw and <body>, inactive (welded to the moving jaw, the
    # part is pressed against the fixed jaw: the jaws stop at its width, as for real). While the gripper is commanded
    # closed and both jaws touch <body>, the weld holds it where it is in the gripper;
    # opening the gripper releases it. MuJoCo's convex-hull jaws otherwise squeeze small
    # round parts out instead of holding them.

    GRIP_OPEN_PCT = 40.0   # commanded opening above which the grip is released

    def _setup_grip_assist(self) -> None:
        m = self.model
        self._grips = []
        self._jaws = (m.body("gripper").id, m.body("moving_jaw_so101_v1").id)
        for i in range(m.nnumeric):
            name = m.numeric(i).name
            if name.startswith("grip_assist:"):
                body = name.removeprefix("grip_assist:")
                self._grips.append({"body": m.body(body).id, "eq": m.equality(f"grip_{body}").id})

    def _weld(self, grip) -> None:
        """Activates the weld with the current gripper -> body pose."""
        d = self.data
        g, b = self.model.eq_obj1id[grip["eq"]], grip["body"]
        R_g = d.xmat[g].reshape(3, 3)
        rel_pos = R_g.T @ (d.xpos[b] - d.xpos[g])
        q_inv = np.zeros(4)
        mujoco.mju_negQuat(q_inv, d.xquat[g])
        rel_quat = np.zeros(4)
        mujoco.mju_mulQuat(rel_quat, q_inv, d.xquat[b])
        data = self.model.eq_data[grip["eq"]]
        data[0:3] = 0.0                  # anchor at body2's origin
        data[3:6] = rel_pos
        data[6:10] = rel_quat
        data[10] = 1.0                   # torquescale
        d.eq_active[grip["eq"]] = 1

    def _update_grip_assist(self) -> None:
        m, d = self.model, self.data
        closing = self._gripper_rad_to_pct(d.ctrl[self._actuator_id[GRIPPER]]) < self.GRIP_OPEN_PCT
        if not closing:
            for grip in self._grips:
                d.eq_active[grip["eq"]] = 0
            return
        if any(d.eq_active[grip["eq"]] for grip in self._grips):
            return   # one object at a time
        for grip in self._grips:   # declaration order = priority (e.g. the cap before the bottle)
            touching = set()
            for c in d.contact[:d.ncon]:
                b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
                if grip["body"] in (b1, b2):
                    touching.add(b2 if b1 == grip["body"] else b1)
            if all(j in touching for j in self._jaws):
                self._weld(grip)
                return

    def _gripper_pct_to_rad(self, pct: float) -> float:
        lo, hi = self._gripper_range
        return lo + np.clip(pct, 0, 100) / 100.0 * (hi - lo)

    def _gripper_rad_to_pct(self, rad: float) -> float:
        lo, hi = self._gripper_range
        return float(np.clip((rad - lo) / (hi - lo) * 100.0, 0, 100))

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("SO101Sim not connected: call connect() first.")
