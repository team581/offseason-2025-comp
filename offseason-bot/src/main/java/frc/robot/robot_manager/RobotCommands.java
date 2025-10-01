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

  public RobotCommands(RobotManager robot, GroundManager groundManager) {
    this.groundManager = groundManager;
    this.robot = robot;

    var requirementsList =
        List.of(groundManager.deploy, groundManager.intake, groundManager.singulator);
    gmRequirements = requirementsList.toArray(Subsystem[]::new);
    requirementsList =
      List.of(
      robot.elevator,
      robot.arm,
      robot.claw,
      robot.climber);
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

  public Command stowCommand() {
    return Commands.runOnce(robot::stowRequest, bothRequirements)
        .andThen(
            Commands.waitUntil(
                () ->
                    robot.elevator.atGoal()
                        && robot.arm.atGoal()
                        && robot.groundManager.deploy.atGoal()))
        .withName("StowCommand");
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

  public Command confirmScoreCommand() {
    return Commands.runOnce(robot::confirmScoreRequest, bothRequirements)
        .withName("ConfirmScoreCommand");
  }

  public Command forcedL1Request() {
    return Commands.runOnce(robot::forcedL1Request, bothRequirements)
        .withName("ForcedSensorCommand");
  }

  public Command forcedLowStowCommand() {
    return Commands.runOnce(robot::forcedLowStowRequest, rmRequirements)
        .withName("ForcedLowStowCommand");
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

  public Command rehomeDeployCommand() {
    return Commands.runOnce(robot.groundManager::rehomeRequest, gmRequirements)
        .withName("RehomeDeployCommand");
  }

  public Command spinToWinCommand() {
    return Commands.runOnce(robot::spinToWinRequest, rmRequirements).withName("SpinToWinCommand");
  }

  public Command lowStowCommand() {
    return Commands.runOnce(robot::lowStowRequest, rmRequirements).withName("LowStowCommand");
  }
}
