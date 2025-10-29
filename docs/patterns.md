# Patterns

How Team 581's software team writes good code.

## Determinism and maintainability

### Determinism through state machines

Determinism makes debugging faster.
Finite state machines are the easiest way to do this.
They make the code ugly but easy to understand.

Commands are succinct but add a lot of "magic" to the code.
They lack the same level of visibility that state machines have.
Commands should only really be used for being used to trigger state machine transition requests.
Long running commands make are especially difficult to understand.

We started doing this in 2024 after being repeatedly burned by command related bugs in prior seasons.

#### Keep state `enum`s simple

Minimize the amount of metadata associated with state machine state `enum`s.
Lean towards adding more states rather than adding in additional fields to the `enum`.
This ensures that we can be 100% certain what code is executing in the state machinejust based on the current state.

We started doing this in 2024, after spending a bunch of time in 2023 debugging robot automation issues related to "hidden" state in the robot manager class.

### Organize code by what it manages

Don't organize code into a `commands` folder, a `subsystems` folder, etc.
Organize it by what it's responsible for (ex. the intake, or localization).
Keep related files close together.

We started doing this in 2022 to make it easier to organize complex codebases.

### Dumb subsystems, smart managers

Keep individual subsystems "dumb".
Mechanism subsystems shouldn't transition states on their own (with exceptions for things like homing states).
They should expose sensor data to a manager subsystem, which handles all state transitions.

Keeping all transition logic in a single manager subsystem makes the code ugly but greatly reduces the complexity of having multiple places where transitions can occur.

#### Don't nest managers

Same as the previous thing, having state transitions occur in multiple places is difficult to reason about.

We learned this in 2024 after experiencing bugs related to coordinating `RobotManager` and `NoteManager`.

One exception to this rule is when the two managers are decoupled enough that it really does make sense to keep them separate.
For example, the 2025 `GroundManager` and `RobotManager`, since intaking coral and scoring L1 is totally independent of the rest of the robot.

### Explicit subsystem execution order

Ensure that subsystems execute in the optimal order.
Determine what the new setpoints are in a manager, then run the child subsystems to send out commands based on that.

We started doing this in 2023 after seeing how unresponsive the robot was as a result of bad subsystem sequencing (ex. two robot loops between when setpoint was updated and when the motor was commanded).

## Performance

### All code running all the time

To prevent performance issues from occurring mid-match, avoid having code run after a specific event (ex. starting the auto only once enabled).

We started doing this in 2025 after seeing how other teams optimized their autos in 2024 for

### Collecting sensor inputs isn't free

Pulling data from sensors takes a small, but non-negligible amount of time.
If you're pulling the same data over and over in a single robot loop, it ends up adding meaningful latency to the robot.

Fetch sensor data once per robot loop and store it until the next loop.
This is done via `StateMachine#collectInputs()`.

We started doing this in the 2024 offseason after seeing performance issues as our automation grew in complexity.

### Minimize memory allocations

Avoid allocating memory by instantiating objects whenever possible.
This means representing angles with `double`s instead of `Rotation2d`s.

We started doing this in 2024 after experiencing major performance issues related to heavy garbage collector pressure.

## Competitions

### Single software laptop

Do all code changes and deploys from a single laptop.
Ensures there is never out-of-date code being deployed.

We started doing this in 2025 after accidentally deploying outdated code which caused a mechanical issue to regress while in the pit.

### Systems check after every change

Run a full systems check after every software change.
Every time.

### No branches

Only use the `main` branch at competitions.

We started doing this in 2023 after accidentally deploying from the `main` branch instead of the branch we had created for the event, causing a bug (which had been fixed on the event branch) to be reintroduced in a match.
