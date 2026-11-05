# Phase 2: bottle decapping

> **Status: design.** The package [`so101_decap`](../ros2_ws/src/tasks/so101_decap) is
> scaffolded: it plugs into the platform, and its parameters and state list are set.
> The states and the YOLO perception are not implemented yet. Choices still open are
> written `[A | B]`.

## Project context

Same platform as phase 1: **SO-101 arm** (5 arm joints + gripper) driven through `/joint_command`, the shared Pinocchio IK and trajectory streaming (`so101_manipulation`), the camera geometry (`so101_vision`), Docker + RViz.

- **Detection:** YOLO (Ultralytics)
- **Inverse kinematics:** Pinocchio CLIK, the same solver as phase 1
- **New hardware:** a fixture that holds and locks the bottle (`[actuated servo clamp | passive holder]`)

**Phase 2 goal:** detect a bottle, pick it, place it in the fixture and lock it, unscrew the cap and put it in a cap box, unlock the bottle, pick it again and put it in a bottle box, then return to idle.

---

## Perception node (`so101_decap/perception_node`, perception container)

Runs on every camera frame.

1. **YOLO detection:** image passed to a YOLO model (`imgsz=[320]`, `conf=[0.5]`). Classes:
   - `bottle`: COCO class, `[pretrained | fine-tuned]` model
   - `cap`: `[fine-tuned on our own labeled images | HSV green threshold inside the bottle bounding box]`
   YOLO predicts boxes and class scores in a single pass; low-confidence boxes are filtered and NMS removes duplicates.
2. **Box → pixel:** for each object, bounding box center `(u, v)` (for the cap, center with `z_plane` = cap height).
3. **Ray in space:** same pinhole model as phase 1:
   `d_cam = ((u - cx) / fx, (v - cy) / fy, 1)`, `d = R · d_cam` via TF.
4. **Plane intersection:** `z_plane = [bottle/cap height above the table]` (the bottle stands upright):
   `t = (z_plane - O_z) / d_z`, `P = O + t · d`.
5. **Publish:** `PoseStamped` on `[/bottle_pose]` and `[/cap_pose]`, frame `base_link`, with the image timestamp.

**Cap-removed check:** the cap is no longer detected on the bottle, combined with the gripper joint stopping at cap width (not fully closed) after grasping.

---

## Inverse kinematics

Same as phase 1: `ArmKinematics.ik_top_down` (top-down orientation, yaw free on a 5-DOF arm, or fixed by `closing_dir` to align the jaws with the neck). Measured limit to keep in mind: a vertical gripper is only reachable up to z ≈ 9 cm. A bottle neck is higher than that, so the neck grasp probably needs a **tilted approach** (`ik_reach`) or a side grasp.

The unscrewing loop needs **no IK**: it moves the `wrist_roll` joint directly in joint space.

---

## Collision avoidance (no MoveIt)

Known obstacles: table, fixture (fixed known pose), cap box, bottle box. Approach:
`[waypoints at clearance heights between fixed poses, as in phase 1 | check every
planned joint waypoint against boxes with Pinocchio + Coal (hpp-fcl) before moving]`.
A carried bottle extends the gripper geometry and must be included in the check.

---

## Task node (`so101_decap/decap_node`): state machine

### Parameters

| Parameter | Value |
|---|---|
| `max_reach` / `min_reach` | `[m]` |
| `move_threshold` / `stale_timeout` | `[m]` / `[s]` |
| `neck_grasp_height` | `[m]` |
| `cap_grasp_height` | `[m]` |
| `fixture_pose` | `[x, y, z]` |
| `cap_box_pose` / `bottle_box_pose` | `[x, y, z]` |
| `unscrew_angle` (per step) | `[rad]` |
| `unscrew_lift` (per step) | `[m]` |
| `max_unscrew_steps` | `[N]` |
| `max_retries` | `[N]` |

### States

**IDLE**
- Arm → `home`, gripper → `open`, fixture unlocked.
- → `DETECT_BOTTLE`

**DETECT_BOTTLE**
- Wait for a fresh `/bottle_pose`, store as `target_pose`.
- → `CHECK_REACH`

**CHECK_REACH**
- Distance within `[min_reach, max_reach]` and IK + plan to the pre-grasp pose succeed.
- Fail → log "bottle out of reach" → `IDLE`
- OK → `APPROACH_BOTTLE`

**APPROACH_BOTTLE**
- Move above the bottle neck; cancel and replan if the bottle moves > `move_threshold`.
- → `GRASP_BOTTLE`

**GRASP_BOTTLE**
- Top-down grasp on the **neck**, just below the cap (only graspable part for a top-down gripper on a standing bottle). Attach the bottle in the planning scene.
- → `TRANSPORT_TO_FIXTURE`

**TRANSPORT_TO_FIXTURE → INSERT**
- Move above `fixture_pose`, slow descent into the fixture, open the gripper, detach, rise.
- → `LOCK`

**LOCK**
- `[Call /fixture/lock (true) and wait for confirmation | passive: no action]`.
- Failure → `RECOVER`
- → `GRASP_CAP`

**GRASP_CAP**
- Descend onto the cap at `cap_grasp_height`, close the gripper.
- → `UNSCREW_LOOP`

**UNSCREW_LOOP** (wrist roll cannot turn endlessly → regrasp loop)
1. Rotate `wrist_roll` by `-unscrew_angle` (counter-clockwise seen from above) while rising by `unscrew_lift`.
2. Check if the cap is free (rises without resistance / gripper still holding).
3. Not free: open gripper → rotate `wrist_roll` back `+unscrew_angle` → close on the cap → repeat.
- Cap free → `LIFT_CAP`
- `max_unscrew_steps` reached → `RECOVER`

**LIFT_CAP → PLACE_CAP_IN_BOX**
- Attach the cap, lift, move above `cap_box_pose`, release, detach.
- Verify the cap is no longer detected on the bottle.
- → `UNLOCK`

**UNLOCK**
- `[Call /fixture/lock (false) | passive: no action]`.
- → `REGRASP_BOTTLE`

**REGRASP_BOTTLE**
- Top-down grasp on the neck again (lower height, the cap is gone). Attach the bottle.
- → `PLACE_BOTTLE_IN_BOX`

**PLACE_BOTTLE_IN_BOX**
- Move above `bottle_box_pose`, descend, release, detach, rise.
- → `HOME`

**HOME**
- Arm → `home`, gripper → `open`.
- → `IDLE`

**RECOVER** (any failure)
- Open the gripper, move up and home, unlock the fixture if locked, report the failure.
- → `IDLE`

### Diagram

```
IDLE -> DETECT_BOTTLE -> CHECK_REACH -(too far)-> IDLE
                              | ok
                       APPROACH_BOTTLE -(moved)-> CHECK_REACH
                              |
                        GRASP_BOTTLE -> TRANSPORT_TO_FIXTURE -> INSERT
                              |
                            LOCK -(failed)-> RECOVER
                              |
                         GRASP_CAP -> UNSCREW_LOOP -(max steps)-> RECOVER
                              | cap free
                     LIFT_CAP -> PLACE_CAP_IN_BOX
                              |
                         UNLOCK -> REGRASP_BOTTLE -> PLACE_BOTTLE_IN_BOX
                              |
                            HOME -> IDLE

RECOVER -> IDLE
```

---

## Known limits

- **Payload:** small STS3215 servos → small, light (empty or near-empty) plastic bottle.
- **Cap torque:** limited gripper force → cap `[pre-loosened]` for the demo.
- **Bottle rotation:** if the fixture does not block rotation, the bottle turns with the cap.
- **Gripper opening:** must fit both the bottle neck and the cap.

---

## Data flow

**Command:** task node → Pinocchio IK → waypoints → `TrajectoryStreamer` → `/joint_command` → `so101_driver` → MuJoCo or Feetech bus (wrist-roll steps of the unscrew loop: joint space, no IK)
**Fixture:** task node → `[/fixture/lock service]` → fixture servo
**State:** `so101_driver` → `/joint_states` → `robot_state_publisher` → TF → RViz / perception