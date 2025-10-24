package com.team581.util.scheduling;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;

public final class SubsystemExecutionSequencer {
  @SuppressWarnings("unchecked")
  private static Set<Command> getScheduledCommands() {
    try {
      var field = CommandScheduler.class.getDeclaredField("m_scheduledCommands");
      field.setAccessible(true);
      var rawResult = field.get(CommandScheduler.getInstance());

      return (Set<Command>) rawResult;
    } catch (NoSuchFieldException | SecurityException | IllegalAccessException e) {
      DogLog.logFault("Failed to do reflection for scheduled commands");
      return Set.of();
    }
  }

  public static RobotMatchState getStage() {
    if (DriverStation.isTeleopEnabled()) {
      return RobotMatchState.TELEOP;
    } else if (DriverStation.isAutonomousEnabled()) {
      return RobotMatchState.AUTONOMOUS;
    } else if (DriverStation.isTestEnabled()) {
      return RobotMatchState.TEST;
    } else {
      return RobotMatchState.DISABLED;
    }
  }

  private static final Queue<PrioritySubsystem> subsystems =
      new PriorityQueue<>(
          Comparator.comparingInt(
                  (PrioritySubsystem subsystem) -> subsystem.getPriority().getValue())
              .reversed());
  private static final CommandScheduler commandScheduler = CommandScheduler.getInstance();
  private static final Set<Command> scheduledCommands = getScheduledCommands();

  public static void ready() {
    for (PrioritySubsystem subsystem : subsystems) {
      commandScheduler.registerSubsystem(subsystem);
    }
  }

  public static void log() {
    DogLog.log(
        "Scheduler/ScheduledCommands",
        scheduledCommands.stream().map(command -> command.getName()).toArray(String[]::new));
  }

  public static void registerSubsystem(PrioritySubsystem subsystem) {
    subsystems.add(subsystem);
    commandScheduler.unregisterSubsystem(subsystem);
  }

  private SubsystemExecutionSequencer() {}
}
