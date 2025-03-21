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
    var requirementsList = List.of(robot.elevator, robot.arm, robot.intake, robot.climber);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command floorIntakeCommand() {
    return Commands.runOnce(robot::intakeFloorCoralHorizontalRequest, requirements).withName("FloorIntakeCommand");
  }

  public Command floorAssistIntakeCommand() {
    return Commands.runOnce(robot::intakeAssistFloorCoralHorizontalRequest, requirements)
        .withName("FloorIntakeCommand");
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

  public Command climbDownCommand() {
    return Commands.runOnce(robot::previousClimbStateRequest, requirements)
        .withName("ClimbDownCommand");
  }

  public Command unjamCommand() {
    return Commands.runOnce(robot::unjamRequest, requirements).withName("UnjamCommand");
  }

  public Command rehomeElevatorCommand() {
    return Commands.runOnce(robot::rehomeElevatorRequest, requirements)
        .withName("RehomeElevatorCommand");
  }

  public Command rehomeArmCommand() {
    return Commands.runOnce(robot::rehomeArmRequest, requirements).withName("RehomeArmCommand");
  }
}
