package frc.robot.robot_manager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.robot_manager.ground_manager.GroundManager;
import java.util.List;

public class RobotCommands {
  private final GroundManager groundManager;
  private final RobotManager robot;
  private final Subsystem[] rmRequirements;
  private final Subsystem[] gmRequirements;
  private final Subsystem[] bothRequirements;

  public RobotCommands(RobotManager robot) {
    this.groundManager = robot.groundManager;
    this.robot = robot;

    var requirementsList =
        List.of(groundManager.deploy, groundManager.intake, groundManager.singulator);
    gmRequirements = requirementsList.toArray(Subsystem[]::new);
    requirementsList = List.of(robot.elevator, robot.arm, robot.claw, robot.climber);
    rmRequirements = requirementsList.toArray(Subsystem[]::new);
    requirementsList =
        List.of(
            robot.elevator,
            robot.arm,
            robot.claw,
            robot.climber,
            robot.groundManager.deploy,
            robot.groundManager.intake);
    bothRequirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command groundIntakeCommand() {
    return Commands.runOnce(groundManager::intakeRequest, gmRequirements)
        .withName("GroundIntakeCoralCommand");
  }

  public Command algaeIntakeGroundCommand() {
    return Commands.runOnce(robot::intakeFloorAlgaeRequest, rmRequirements)
        .withName("AlgaeIntakeGroundCommand");
  }

  public Command outtakeRequestCommand() {
    return Commands.runOnce(robot.groundManager::outtakeRequest, gmRequirements)
        .withName("OuttakeRequestCommand");
  }

  public Command stowCommand() {
    return Commands.runOnce(robot::stowRequest, bothRequirements).withName("StowCommand");
  }

  public Command forceNextScoreSequenceCommand() {
    return Commands.runOnce(robot::forceNextScoringStateRequest, bothRequirements)
        .withName("ForceNextScoreSequenceCommand");
  }

  public Command highLineupCommand() {
    return Commands.runOnce(robot::highLineupRequest, bothRequirements)
        .withName("HighLineupCommand");
  }

  public Command l3LineupCommand() {
    return Commands.runOnce(robot::l3CoralApproachRequest, bothRequirements)
        .withName("L3LineupCommand");
  }

  public Command l2LineupCommand() {
    return Commands.runOnce(robot::l2CoralApproachRequest, bothRequirements)
        .withName("L2LineupCommand");
  }

  public Command lowLineupCommand() {
    return Commands.runOnce(robot::lowLineupRequest, bothRequirements).withName("LowLineupCommand");
  }

  public Command algaeReefIntakeCommand() {
    return Commands.runOnce(robot::algaeReefIntakeRequest, rmRequirements)
        .withName("AlgaeReefIntakeCommand");
  }

  public Command scoreCommand() {
    return Commands.runOnce(robot::scoreRequest, bothRequirements).withName("ConfirmScoreCommand");
  }

  public Command forcedHandoffCommand() {
    return Commands.runOnce(robot::forcedHandoffRequest, rmRequirements)
        .withName("ForcedHandoffCommand");
  }

  public Command scoringAlignOffCommand() {
    return Commands.runOnce(robot::scoringAlignOffRequest, bothRequirements)
        .withName("ScoringAlignOffCommand");
  }

  public Command climbUpCommand() {
    return Commands.runOnce(robot::nextClimbStateRequest, rmRequirements)
        .withName("ClimbUpCommand");
  }

  public Command climbStopCommand() {
    return Commands.runOnce(robot::stopClimbStateRequest, rmRequirements)
        .withName("ClimbStopCommand");
  }

  public Command unjamCommand() {
    return Commands.runOnce(robot::unjamRequest, bothRequirements).withName("UnjamCommand");
  }

  public Command stopOuttakeRequest() {
    return Commands.runOnce(robot::stopOuttakeRequest, gmRequirements)
        .withName("StopOuttakeRequestCommand");
  }

  public Command rehomeDeployCommand() {
    return Commands.runOnce(robot.groundManager::rehomeRequest, gmRequirements)
        .withName("RehomeDeployCommand");
  }

  public Command rehomeElevatorCommand() {
    return Commands.runOnce(robot::rehomeElevatorCommand, rmRequirements)
        .withName("LowStowCommand");
  }
}
