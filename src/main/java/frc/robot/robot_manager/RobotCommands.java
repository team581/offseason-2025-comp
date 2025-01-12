package frc.robot.robot_manager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.autos.trailblazer.Trailblazer;
import java.util.List;

public class RobotCommands {
  private final RobotManager robot;
  private final Subsystem[] requirements;
  private final Trailblazer trailblazer;

  public RobotCommands(RobotManager robot, Trailblazer trailblazer) {
    this.robot = robot;
    this.trailblazer = trailblazer;
    // TODO: Add climber to the requirement list when it is added into robot manager
    var requirementsList = List.of(robot.elevator, robot.wrist, robot.pivot, robot.intake);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }

  // public Command floorIntakeCommand() {
  //   return Commands.runOnce(robot::intakeFloorRequest, requirements)
  //       .andThen(robot.waitForState());
  // }

  public Command confirmScoreCommand() {
    return Commands.runOnce(robot::confirmScoreRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command climbUpCommand() {
    return Commands.runOnce(robot::nextClimbStateRequest, requirements);
  }

  public Command climbDownCommand() {
    return Commands.runOnce(robot::previousClimbStateRequest, requirements);
  }
}
