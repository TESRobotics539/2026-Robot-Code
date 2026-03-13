# CLAUDE.md — FRC 539 (Helga the Viking Princess)

## Global Rules
- Always use **feet and inches** for all units in responses, comments, and explanations. Never use metric.
- Before proposing a non-trivial new implementation or architecture, check frc5687's GitHub codebases for prior art: https://github.com/frc5687/ (repos named `YYYY-robot`, 2020–2026). Prefer battle-tested patterns over novel designs.

## FRC Reference Teams
Teams worth checking for prior art: **5687** (Outliers), **1678** (Citrus Circuits), **254** (Cheesy Poofs), **4481** (Rembrandts), **Lynk Robotics**, **971** (Spartan Robotics)

Key patterns:
- IO layer (hardware abstraction) — universally adopted; `RealIO` / `SimIO` / `NoopIO` per subsystem
- `SubsystemIO<Inputs, Outputs>` with `process()` pipeline: read hw → log → logic → log → write hw (5687 2025)
- `TunableDouble` via NetworkTables for live PID tuning without redeploy (5687); `hasChanged()` triggers reconfiguration
- COM-aware swerve velocity limiting based on elevator height — 4-point linear interpolation, clamped [0.1, 1.0] (5687)
- `COMSwerveSetpointGenerator` decorator wraps PathPlanner generator to inject height-based constraint scaling (5687)
- Flag-based superstructure interlocks: `pivotCanMove = elevator.isNear(goal, tolerance)` (971/5687)
- `SuperstructureState` with `Optional<SubsystemState>` fields — only move subsystems with a value present (5687)
- `RequestHandler` with IMMEDIATE / QUEUED / AUTO_SEQUENCE types; QUEUED waits for `Supplier<Boolean> driveCondition` (5687)
- `ConcurrentTimeInterpolatableBuffer` for vision latency compensation (254)
- `VisionSTDFilter` dynamic stddevs: single-tag 2× penalty, multi-tag 0.2×, distance-penalized, confidence-scaled (5687)
- Probabilistic game-piece tracker: `probability *= 0.95` decay per cycle; detections within 0.3m merged (5687)
- `tanh`-based static friction feedforward: `KG + KV*v + KA*a + tanh(150*error)*KS` (4481)
- High-frequency odometry thread at 250 Hz with `ReadWriteLock` (254/5687)
- Separate vision pipelines: pose estimation vs. game-piece detection (1678/5687)
- `ChoreoVariableWriter` to inject field geometry at startup for alliance-agnostic paths (4481)
- `LoggedTracer` for per-subsystem loop time profiling (1678)
- `InterpolatingTreeMap<K,V>` for distance-to-setpoint lookups — binary search + linear interpolation on tested data points (5687/254)
- Reference-tracking feedforward: `Kff * (next_R - A*R)` using plant dynamics (971)
- IntegralFlywheel: augment state vector with voltage bias state (971)
- `ControlState` enum on DriveTrain: `NEUTRAL`, `MANUAL`, `POSITION`, `ROTATION`, `TRAJECTORY` — prevents conflicting control modes (5687)
- Encoder drift auto-correction: if `|relativePos - absolutePos| > threshold`, re-sync relative encoder in `periodic()` (5687)
- Dual-motor position watchdog: compare east/west motor positions; trip safety if delta exceeds limit (5687)
- `MotionMagicExponentialTorque` for smoother position profiles vs. standard trapezoidal (5687)
- `ConditionalCommand(modeACmd, modeBCmd, subsystem::isModeA)` for mode-switching button bindings (5687)
- `.finallyDo()` / `.handleInterrupt()` on every button binding for guaranteed cleanup (5687)
- `KillAll` emergency stop command bound to a dedicated button — interrupts all subsystems (5687)
- `ZeroSubsystem` command: explicit zeroing as a command, not just an init method — call before enabling mechanisms (5687)
- `RumbleGamepad(controller, intensity, seconds)` as a command — chain with `andThen()` for haptic feedback (5687)
- `withoutLimitsTemporarily()` on servo subsystems — disables soft limits for command duration; clean homing pattern (254)
- `Commands.defer(supplier, Set.of(subsystems))` for runtime-branched collision avoidance — branches evaluated at execution time, not construction (1678)
- `BeamBreakIO.stateWaitWithDebounceIfReal(bool, simSeconds)` — sim-safe sensor waiting; real robot blocks on sensor, sim waits fixed duration (1678)
- Drive/superstructure sync flags: `setDriveReady()`, `getSuperstructureDone()`, `setReadyToRaiseElevator()` — `FollowSyncedPIDToPose.isFinished()` returns `superstructureDone` (1678)
- `Drive.getStable()` scoring gate: checks pitch, roll, translational, rotational speed against thresholds before allowing scoring (1678)
- `ScheduleIfWontCancelOther` — schedules a command only if all required subsystems are currently free (1678)
- Velocity-lookahead branch selection: project robot position 100 ms forward at current velocity to pre-select nearest branch (1678)
- `RobotController.setBrownoutVoltage(Units.Volts.of(4.6))` in `robotInit()` (1678)
- Thread priority: `odometryThread.setThreadPriority(31)` + `Threads.setCurrentThreadPriority(true, 4)` in `robotPeriodic()` (1678/254)
- `GcStatsCollector`: queries `GarbageCollectorMXBeans`, publishes GC time and count each loop (1678)
- `FollowSyncedTagPIDToPose`: rumble controller if drive arrives at target but no recent AprilTag estimate (1678)
- `AtomicBoolean` falling-edge detection inside `until()` lambdas — clean sensor transition without side effects (254)
- `Supplier<Setpoint>` parameterized factory commands — live-updating setpoints at execution time, not command creation (254)
- Auto warmup during disabled: run dummy trajectory/pathfinding command to pre-JIT; reschedule if it completes early (254/4481)
- Three-variant tuner constants: `hasMacAddress(compMac)` → comp vs. practice; `Robot.isSimulation()` → sim (254)
- `SignalLogger.enableAutoLogging(false)` — suppresses CTRE built-in logger to prevent double-logging with AdvantageKit (254)
- `SelectCommand<State>` with `Map.ofEntries()` for multi-mode command dispatch — scales to 11+ states, no if/else chains (4481)
- `TunableOption implements BooleanSupplier` — feature flags via SmartDashboard, composable directly with `Trigger.and()` / `Commands.either()` (Lynk)
- `LoggedCommands` drop-in `Commands` factory replacement — same API, every method logs init/end to DogLog (Lynk)
- `hasAllianceTag(List<Short> ids)` — reject opposite-alliance AprilTags by field X position split (Lynk)
- `flipIfRed(Translation2d)` — normalize all field positions to blue-origin before zone math (Lynk)
- Vision stddev: `distance² / tagCount` scaling — both applied together; per-camera-mode multiplier (Lynk)
- `CalibrationTable<T extends Interpolatable<T>>` — generic interpolating lookup, works for any `Interpolatable<T>` (Lynk)
- Collision avoidance via goal clamping: `elevator.setGoal(MathUtil.clamp(goal, avoidance.getMin(), avoidance.getMax()))` — clamp subsystem goals to dynamic safe ranges rather than blocking commands (971)
- `ZeroingState` machine: `UNINITIALIZED → ZEROING → RUNNING`; reduced voltage during zeroing, full voltage during `RUNNING`; apply absolute encoder offset average on transition (971)
- 3-iteration virtual target: `for (i<3) { airTime = dist/speed; virtualTarget = target − velocity*airTime; }` — motion-compensated shoot-on-the-fly (971/5687)
- `DeweightAprilTag(id)` — per-tag stddev multiplier to reduce confidence in unreliable field tags (971)
- Dual-model localizer for tip detection: parallel model-branch (encoders) + accel-branch (IMU); high divergence → switch to accel-based tracking (971 y2022)



## Tech Stack
- Java 17, Gradle/GradleRIO, WPILib 2026, AdvantageKit 26.0.0, CTRE Phoenix 6 (26.1.0), REVLib (2026.0.1), PathPlanner (2026.1.2)

## WPILib Command Composition Semantics

| Method | Group Type | Ends When |
|--------|-----------|-----------|
| `alongWith()` | `ParallelCommandGroup` | ALL commands finish |
| `raceWith()` | `ParallelRaceGroup` | ANY command finishes |
| `deadlineFor()` | `ParallelDeadlineGroup` | The deadline (calling) command finishes, interrupts all others |
| `andThen()` | `SequentialCommandGroup` | Commands run in order, ends when last finishes |

| Factory / Class | Behavior |
|----------------|---------|
| `InstantCommand` / `runOnce()` | Runs once, finishes immediately |
| `RunCommand` / `run()` | Runs every cycle, never finishes on its own |
| `runEnd()` | Runs every cycle with an end action, never finishes on its own |
| `startEnd()` | Runs a start action on init; runs an end action when interrupted, never finishes on its own |

### Common pitfalls
- **PathPlanner NamedCommands must terminate** — an infinite command (e.g., `run()`) will stall the entire auto sequence
- **`FieldCentricFacingAngle.HeadingController.atSetpoint()`** returns stale/invalid results when the facing-angle request is not actively being applied; ensure that it's being reset when called
- **CTRE `StatusSignal` values must be refreshed** before reading; stale signals return old data silently

## Code Patterns
- Command-based: superstructure owns non-drivetrain subsystems; subsystems own hardware; commands define actions
- Superstructure state transition commands are fire-and-forget
- Subsystems can be unhooked from superstructure and run directly by commands for testing
- **Infrastructure constants** (CAN IDs, ports, hardware config) → `Constants.java`
- **Tuning constants** (PID gains, setpoints, tolerances) → `private static final` in the file where used
- Avoid unnamed literals — use descriptive names with units (e.g., `MAX_VELOCITY_MPS`, `STALL_CURRENT_AMPS`)
- Use AdvantageKit `@AutoLog` on IO input classes; `Logger.recordOutput()` for all other values grouped by subsystem
- Subsystems extend `SubsystemBase`; subsystems have IO layers (except superstructure) following 6328's architecture
- Every robot config in `RobotContainer` must initialize all subsystems — use Noop IO for missing hardware

## Naming Conventions

### States (Enum Values)
- `ALL_CAPS` with underscores
- **Present participles** for action states: `INTAKING`, `SHOOTING`, `SPINNING_UP`, `EJECTING`
- **Adjectives/nouns** for condition states: `IDLE`, `READY_TO_FIRE`, `AT_SETPOINT`, `DISABLED`
- Always spell out words fully (no "2" for "TO", "4" for "FOR")

### Commands
- **Command classes**: PascalCase with `Command` suffix — `FlywheelTuneCommand`, `DriveToPositionCommand`
- **Action methods**: camelCase verb-first — `shootWithSpinup()`, `alignToTarget()`; set-prefix for config: `setFlywheelSpeed()`
- Avoid "Cmd" abbreviation — use full "Command" suffix or drop it entirely

### Classes
- **Subsystems**: PascalCase nouns, no "Subsystem" suffix — `Intake`, `Flywheel`, `SuperStructure`
- **IO interfaces**: `<Subsystem>IO` — `IntakeIO`, `FlywheelIO`
- **IO implementations**: `<Subsystem>IO<Type>` — `IntakeIOSim`, `FlywheelIOSparkMax`
- **Utility classes**: descriptive purpose + type suffix — `ShotCalculator`, `AllianceFlipUtil`

## PR Review Guidelines
- **Flag breaking changes only** — not style, naming, or nitpicks
- **Only review lines in the PR diff** — do not flag pre-existing issues
- CI blockers: Build, Test, Format Check, Simulation Test only

### Breaking changes to flag
- Command lifecycle changes, removed cleanup/stop actions, changed motor inversions or sensor polarity
- Cross-config impact on shared code (SuperStructure, RobotContainer, command factories)
- Hardcoded sensor overrides that gate state transitions or safety logic
- CAN ID conflicts on the same bus
- Disabled periodic calculations, safety checks, or sensor reads
- API signature changes (renamed/removed public methods)

### Not worth flagging
- Style, formatting, or naming preferences
- Missing Javadoc or comments
- Unnamed numeric literals in test/tuning bindings
- Code organization choices

## Do
- Write clear, commented code
- Add Javadoc to public methods
- Log important values with AdvantageKit
- Use units consistently in variable names (e.g., `_MPS`, `_AMPS`, `_SECONDS`)

## Don't
- Hard-code CAN IDs or port numbers (use `Constants.java`)
- Use unnamed numeric literals
- Leave any subsystem uninitialized in `RobotContainer` — use a Noop IO if hardware is absent
- Delete existing tests
- Modify vendor dependencies without team discussion

## Ask First
- Before adding new dependencies
- Before restructuring folders
- Before changing CAN IDs or motor configurations
- Before modifying autonomous routines
