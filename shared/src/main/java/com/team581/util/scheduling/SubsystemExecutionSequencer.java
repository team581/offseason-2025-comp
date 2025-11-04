package com.team581.util.scheduling;

import static java.util.Comparator.comparingInt;

import com.google.common.collect.ImmutableSet;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
      return ImmutableSet.of();
    }
  }

  public static RobotMatchState getStage() {
    if (DriverStation.isTeleopEnabled()) {
      return RobotMatchState.TELEOP;
    } else if (DriverStation.isAutonomousEnabled()) {
      return RobotMatchState.AUTONOMOUS;
    } else {
      return RobotMatchState.DISABLED;
    }
  }

  private static final Queue<PrioritySubsystem> SUBSYSTEMS =
      new PriorityQueue<>(
          comparingInt((PrioritySubsystem subsystem) -> subsystem.getPriority().getValue())
              .reversed());
  private static final CommandScheduler COMMAND_SCHEDULER = CommandScheduler.getInstance();

  @SuppressWarnings("ImmutableMemberCollection")
  private static final Set<Command> SCHEDULED_COMMANDS = getScheduledCommands();

  public static void ready() {
    for (PrioritySubsystem subsystem : SUBSYSTEMS) {
      COMMAND_SCHEDULER.registerSubsystem(subsystem);
    }
  }

  public static void log() {
    DogLog.log(
        "Scheduler/ScheduledCommands",
        SCHEDULED_COMMANDS.stream().map(command -> command.getName()).toArray(String[]::new));
  }

  public static void registerSubsystem(PrioritySubsystem subsystem) {
    SUBSYSTEMS.add(subsystem);
    COMMAND_SCHEDULER.unregisterSubsystem(subsystem);
  }

  private SubsystemExecutionSequencer() {}
}
