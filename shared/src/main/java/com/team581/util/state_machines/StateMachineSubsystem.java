package com.team581.util.state_machines;

import com.team581.util.scheduling.PrioritySubsystem;
import com.team581.util.scheduling.RobotMatchState;
import com.team581.util.scheduling.SubsystemExecutionSequencer;
import com.team581.util.scheduling.SubsystemPriorityBase;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import org.jspecify.annotations.Nullable;

/**
 * A state machine that is also a subsystem. Extends {@link StateMachine} and implements {@link
 * Subsystem}.
 */
public class StateMachineSubsystem<S extends Enum<S>> extends StateMachine<S>
    implements PrioritySubsystem {
  public static String getSubsystemName(Class<?> cls) {
    var name = cls.getSimpleName();

    name = name.substring(name.lastIndexOf('.') + 1);
    if (name.endsWith("Subsystem")) {
      name = name.substring(0, name.length() - "Subsystem".length());
    }

    return name;
  }

  private static final StateMachineSubsystemInputManager MANAGER =
      new StateMachineSubsystemInputManager();

  private final SubsystemPriorityBase priority;

  private final String loggerName;

  protected final String subsystemName;

  private @Nullable RobotMatchState previousStage = null;

  /**
   * Creates a new state machine subsystem.
   *
   * @param priority The subsystem priority of this subsystem in {@link
   *     SubsystemExecutionSequencer}.
   * @param initialState The initial/default state of the state machine.
   */
  protected StateMachineSubsystem(SubsystemPriorityBase priority, S initialState) {
    super(initialState);

    this.priority = priority;

    SubsystemExecutionSequencer.registerSubsystem(this);

    subsystemName = getSubsystemName(getClass());
    loggerName = "Scheduler/Subsystems/" + subsystemName + ".periodic()";

    MANAGER.register(this);
  }

  @Override
  public SubsystemPriorityBase getPriority() {
    return priority;
  }

  /** {@link IterativeRobotBase#robotPeriodic()} */
  public void robotPeriodic() {
    super.periodic();
  }

  /** {@link IterativeRobotBase#autonomousInit()} */
  public void autonomousInit() {}

  /** {@link IterativeRobotBase#autonomousPeriodic()} */
  public void autonomousPeriodic() {}

  /** {@link IterativeRobotBase#teleopInit()} */
  public void teleopInit() {}

  /** {@link IterativeRobotBase#teleopPeriodic()} */
  public void teleopPeriodic() {}

  /** {@link IterativeRobotBase#disabledInit()} */
  public void disabledInit() {}

  /** {@link IterativeRobotBase#disabledPeriodic()} */
  public void disabledPeriodic() {}

  @Override
  public void periodic() {
    DogLog.time(loggerName);

    RobotMatchState stage = SubsystemExecutionSequencer.getStage();

    boolean isInit = previousStage != stage;

    robotPeriodic();

    switch (stage) {
      case DISABLED -> {
        if (isInit) {
          disabledInit();
        }

        disabledPeriodic();
      }
      case TELEOP -> {
        if (isInit) {
          teleopInit();
        }

        teleopPeriodic();
      }
      case AUTONOMOUS -> {
        if (isInit) {
          autonomousInit();
        }

        autonomousPeriodic();
      }
    }

    DogLog.timeEnd(loggerName);

    previousStage = stage;
  }

  /**
   * Creates a command that waits until this state machine is in the given state.
   *
   * @param goalState The state to wait for.
   * @return A command that waits until the state is equal to the goal state.
   */
  public Command waitForState(S goalState) {
    return Commands.waitUntil(() -> this.getState() == goalState);
  }

  /**
   * Creates a command that waits until this state machine is in any of the given states.
   *
   * @param goalStates A set of the states to wait for.
   * @return A command that waits until the state is equal to any of the goal states.
   */
  public Command waitForStates(Set<S> goalStates) {
    return Commands.waitUntil(() -> goalStates.contains(this.getState()));
  }

  /**
   * Creates a command that waits until this state machine is in any of the given states.
   *
   * @param goalStates An array of the states to wait for.
   * @return A command that waits until the state is equal to any of the goal states.
   */
  @SafeVarargs
  public final Command waitForStates(S... goalStates) {
    return waitForStates(Set.of(goalStates));
  }
}
