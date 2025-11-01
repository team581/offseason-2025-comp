package frc.robot.robot_manager;

import com.google.common.collect.ImmutableList;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RobotCommands {
  private final RobotManager robot;
  private final Subsystem[] requirements;

  public RobotCommands(RobotManager robot) {
    this.robot = robot;

    var requirementsList = ImmutableList.of();
    requirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command rehomeWristCommand() {
    return Commands.runOnce(robot::rehomeWristRequest, requirements).withName("RehomeWristCommand");
  }

  public Command rehomeElevatorCommand() {
    return Commands.runOnce(robot::rehomeElevatorRequest, requirements)
        .withName("RehomeElevatorCommand");
  }

  public Command unjamCommand() {
    return Commands.runOnce(robot::unjamRequest, requirements).withName("UnjamCommand");
  }

  public Command intakeCoralCommand() {
    return Commands.runOnce(robot::intakeCoralRequest, requirements).withName("IntakeCoralCommand");
  }

  public Command algaeGroundIntakeCommand() {
    return Commands.runOnce(robot::intakeGroundAlgaeRequest, requirements)
        .withName("AlgaeGroundIntakeCommand");
  }

  public Command algaeReefIntakeCommand() {
    return Commands.runOnce(robot::algaeReefIntakeRequest, requirements)
        .withName("AlgaeReefIntakeCommand");
  }

  public Command netWaitCommand() {
    return Commands.runOnce(robot::netWaitRequest, requirements).withName("NetWaitCommand");
  }

  public Command lowLineupCommand() {
    return Commands.runOnce(robot::lowLineupRequest, requirements).withName("LowLineupCommand");
  }

  public Command stowCommand() {
    return Commands.runOnce(robot::stowRequest, requirements).withName("StowCommand");
  }

  public Command confirmScoreCommand() {
    return Commands.runOnce(robot::confirmScoreRequest, requirements)
        .withName("ConfirmScoreRequest");
  }

  public Command scoringAlignOffCommand() {
    return Commands.runOnce(robot::scoringAlignOffRequest, requirements)
        .withName("ScoringAlignOffCommand");
  }

  public Command climberSequenceForwardCommand() {
    return Commands.runOnce(robot::climberSequenceForward, requirements)
        .withName("ClimberSequenceForwardCommand");
  }

  public Command climberSequenceStopCommand() {
    return Commands.runOnce(robot::climberSequenceStop, requirements)
        .withName("ClimberSequenceStopCommand");
  }
}
