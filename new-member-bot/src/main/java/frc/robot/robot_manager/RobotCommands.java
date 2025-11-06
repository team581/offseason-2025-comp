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

  public Command fullStowCommand() {
    return Commands.runOnce(robot::fullStowRequest, requirements).withName("FullStowCommand");
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

  public Command climberSequenceBackwardCommand() {
    return Commands.runOnce(robot::climberSequenceBackward, requirements)
        .withName("ClimberSequenceBackwardCommand");
  }

  public Command outtakeCoral() {
    return Commands.runOnce(robot::outtakeCoralRequest, requirements)
        .withName("OuttakeCoralCommand");
  }

  public Command outtakeAlgae() {
    return Commands.runOnce(robot::outtakeAlgaeRequest, requirements)
        .withName("OuttakeAlgaeCommand");
  }
}
