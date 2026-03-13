# CLAUDE.md — FRC 539 (Helga the Viking Princess)

## Global Rules
- Always use **feet and inches** for all units in responses, comments, and explanations. Never use metric.
- Before proposing a non-trivial new implementation or architecture, check frc5687's GitHub codebases for prior art: https://github.com/frc5687/ (repos named `YYYY-robot`, 2020–2026). Prefer battle-tested patterns over novel designs.

## FRC Reference Teams
Teams worth checking for prior art: **5687** (Outliers), **1678** (Citrus Circuits), **254** (Cheesy Poofs), **4481** (Rembrandts), **Lynk Robotics**, **971** (Spartan Robotics)

Key patterns:
- IO layer (hardware abstraction) — universally adopted; `RealIO` / `SimIO` / `NoopIO` per subsystem
- `TunableDouble` via NetworkTables for live PID tuning without redeploy (5687 pattern)
- COM-aware swerve velocity limiting based on elevator height (5687)
- Flag-based superstructure interlocks: `pivotCanMove = elevator.isNear(goal, tolerance)` (971/5687)
- `ConcurrentTimeInterpolatableBuffer` for vision latency compensation (254)
- `tanh`-based static friction feedforward: `KG + KV*v + KA*a + tanh(150*error)*KS` (4481)
- High-frequency odometry thread at 250 Hz with `ReadWriteLock` (254)
- Separate vision pipelines: pose estimation vs. game-piece detection (1678)
- `ChoreoVariableWriter` to inject field geometry at startup for alliance-agnostic paths (4481)
- `LoggedTracer` for per-subsystem loop time profiling (1678)
- Reference-tracking feedforward: `Kff * (next_R - A*R)` using plant dynamics (971)
- IntegralFlywheel: augment state vector with voltage bias state (971)



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
