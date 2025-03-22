package frc.robot.robot_manager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;

public class RobotCommands {
  private final RobotManager robot;
  private final Subsystem[] requirements;

  public RobotCommands(RobotManager robot) {
    this.robot = robot;
    var requirementsList = List.of(robot.elevator, robot.arm, robot.claw, robot.climber);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command coralGroundIntakeCommand() {
    return Commands.runOnce(robot::intakeFloorCoralHorizontalRequest, requirements)
        .withName("CoralGroundIntakeCommand");
  }

  public Command algaeIntakeGroundCommand() {
    return Commands.runOnce(robot::intakeFloorAlgaeRequest, requirements)
        .withName("AlgaeIntakeGroundCommand");
  }

  public Command floorAssistIntakeCommand() {
    return Commands.runOnce(robot::intakeAssistFloorCoralHorizontalRequest, requirements)
        .withName("FloorIntakeCommand");
  }

  public Command highLineupCommand() {
    return Commands.runOnce(robot::highLineupRequest, requirements).withName("HighLineupCommand");
  }

  public Command l3LineupCommand() {
    return Commands.runOnce(robot::l3LineupRequest, requirements).withName("L3LineupCommand");
  }

  public Command l2LineupCommand() {
    return Commands.runOnce(robot::l2LineupRequest, requirements).withName("L2LineupCommand");
  }

  public Command lowLineupCommand() {
    return Commands.runOnce(robot::lowLineupRequest, requirements)
        .withName("LowLineupCommand");
  }

  public Command algaeReefIntakeCommand() {
    return Commands.runOnce(robot::algaeReefIntakeRequest, requirements)
        .withName("AlgaeReefIntakeCommand");
  }

  public Command confirmScoreCommand() {
    return Commands.runOnce(robot::confirmScoreRequest, requirements)
        .withName("ConfirmScoreCommand");
  }

  public Command stowCommand() {
    return Commands.runOnce(robot::stowRequest, requirements)
        .andThen(Commands.waitUntil(() -> robot.elevator.atGoal() && robot.arm.atGoal()))
        .withName("StowCommand");
  }

  public Command climbUpCommand() {
    return Commands.runOnce(robot::nextClimbStateRequest, requirements).withName("ClimbUpCommand");
  }

  public Command climbStopCommand() {
    return Commands.runOnce(robot::stowRequest, requirements).withName("ClimbStopCommand");
  }

  public Command unjamCommand() {
    return Commands.runOnce(robot::unjamRequest, requirements).withName("UnjamCommand");
  }

  public Command rehomeElevatorCommand() {
    return Commands.runOnce(robot::rehomeElevatorRequest, requirements)
        .withName("RehomeElevatorCommand");
  }

  public Command rehomeDeployCommand() {
    return Commands.runOnce(robot::rehomeDeployRequest, requirements)
        .withName("RehomeDeployCommand");
  }
}
