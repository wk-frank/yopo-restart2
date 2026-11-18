# restart-yopo

Autonomous multi-rotor obstacle traversal and formation coordination across deterministic simulation (L0), lockstep PX4/Gazebo SITL, and onboard Jetson Orin Nano deployment.

## Language

### Perception & Safety Space

**Tri-State Depth Mask（三态深度掩码）**:
A per-pixel classification of depth into `ObservedSurface` (finite return within reliable range), `VerifiedFarFree` (`+inf` return confirmed beyond far clip), and `UnknownVoid` (`NaN` from occlusion, stereo shadow, exposure saturation, `< MinZ` blind zone, or out-of-FOV border). Safety checks treat `UnknownVoid` as untraversable (`known = False`).
_Avoid_: Hole, invalid depth, NaN free space

**Reliable Clearance Horizon（可靠净视距, RCH）**:
The maximum forward swept distance at which the onboard depth camera detects the thinnest target obstacle under operating lighting and motion with validated error bounds. RCH bounds the maximum allowable cruise speed.
_Avoid_: Far clip distance, nominal sensor range

**Verified Swept Neighbourhood（已验证机体扫掠邻域）**:
The immediate sphere of radius `body_radius_m` currently occupied by the aircraft structure, excluding unverified lateral space outside the camera frustum.
_Avoid_: Unconditional 1.2 m camera-origin free sphere

### Multi-Aircraft Coordination

**Scheduled Slot Formation（刚性时刻表槽位编队）**:
Open-area coordination where each aircraft tracks a fixed body-frame slot attached to a pre-integrated virtual center curve (`_CenterPath`) under four-phase reservation voting (`FormationFleet`, `policy=formation`).
_Avoid_: Generic formation

**Elastic Line Formation（弹性横队）**:
Centerless forward coordination where aircraft launch from a common line, replan independently around obstacles using local depth, bound longitudinal spread via `COHESION_BRAKE` (`max_progress_span_m`), and gently return toward initial lateral lanes via clearance-adaptive `lateral_gain` (`ExploreFleet`, `policy=explore`).
_Avoid_: Rigid formation with obstacle avoidance

**Dual-Track Occupancy Union（双轨占用并集）**:
The spatial union of a neighbor's previous committed trajectory (including its braking tail and terminal hold) and its newly received trajectory during the wireless propagation uncertainty window $\delta_{\text{net}}$.
A shared-space commitment also requires a fresh measured position for each peer and separation from its physically reachable set through the braking tail, including an unheard replan.
_Avoid_: Synchronous ACK barrier, narrow-corridor mutex

### Control Handover

**CTBR Shadow Mode（CTBR 影子并跑模式）**:
A verification mode where `TrajectorySetpoint` (PVA) retains sole flight-control authority while `FlatnessCTBRController` computes collective thrust and body rates in parallel for telemetry comparison.
_Avoid_: Dual-output mixed control

**Latched Terminal Handover（终点单向锁存接管）**:
The irreversible transition from CTBR (`VehicleRatesSetpoint`) back to PVA (`TrajectorySetpoint`) once `_terminal_goal` is locked, preventing replanning from re-enabling body-rate control during braking and hover hold.
_Avoid_: Code-triggered mode toggling
