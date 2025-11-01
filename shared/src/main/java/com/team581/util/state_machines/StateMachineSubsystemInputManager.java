package com.team581.util.state_machines;

import static java.util.Comparator.comparingInt;

import com.team581.util.scheduling.PrioritySubsystem;
import com.team581.util.scheduling.SubsystemPriorityBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.PriorityQueue;
import java.util.Queue;

/** Helps ensure that state machines can collect inputs before executing state actions. */
public class StateMachineSubsystemInputManager extends SubsystemBase implements PrioritySubsystem {
  // Sort by lowest priority first
  private final Queue<StateMachineSubsystem<?>> stateMachineSubsystems =
      new PriorityQueue<>(comparingInt(stateMachine -> stateMachine.getPriority().getValue()));

  @Override
  public SubsystemPriorityBase getPriority() {
    // Ensures that state machine inputs are gathered at the right time
    // Subsystem inputs are collected in reverse order of priority (so lowest priority first)
    return () -> 999;
  }

  public void register(StateMachineSubsystem<?> stateMachine) {
    stateMachineSubsystems.add(stateMachine);
  }

  @Override
  public void periodic() {
    for (var stateMachineSubsystem : stateMachineSubsystems) {
      stateMachineSubsystem.beforePeriodic();
    }
  }
}
