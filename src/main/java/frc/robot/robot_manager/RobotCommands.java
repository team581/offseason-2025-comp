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
    var requirementsList =
        List.of(robot.elevator, robot.wrist, robot.roll, robot.intake, robot.climber);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command floorIntakeCommand() {
    return Commands.runOnce(robot::intakeFloorRequest, requirements);
  }

  public Command confirmScoreCommand() {
    return Commands.runOnce(robot::confirmScoreRequest, requirements);
  }

  public Command stowCommand() {
    return Commands.runOnce(robot::stowRequest, requirements)
        .andThen(
            robot.waitForStates(
                RobotState.IDLE_ALGAE, RobotState.IDLE_CORAL, RobotState.IDLE_NO_GP));
  }

  public Command intakeStationCommand() {
    return Commands.runOnce(robot::intakeStationRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_CORAL));
  }

  public Command lowLineupCommand() {
    return Commands.runOnce(robot::lowLineupRequest, requirements);
  }

  public Command l2LineupCommand() {
    return Commands.runOnce(robot::l2LineupRequest, requirements);
  }

  public Command l3LineupCommand() {
    return Commands.runOnce(robot::l3LineupRequest, requirements);
  }

  public Command highLineupCommand() {
    return Commands.runOnce(robot::highLineupRequest, requirements);
  }

  public Command setGamepieceModeCommand(GamePieceMode newMode) {
    return Commands.runOnce(
        () -> {
          robot.setGamePieceMode(newMode);
        });
  }

  public Command climbUpCommand() {
    return Commands.runOnce(robot::nextClimbStateRequest, requirements);
  }

  public Command climbDownCommand() {
    return Commands.runOnce(robot::previousClimbStateRequest, requirements);
  }

  public Command unjamCommand() {
    return Commands.runOnce(robot::unjamRequest, requirements);
  }

  public Command reHomeCommand() {
    return Commands.runOnce(robot::rehomeRequest, requirements);
  }
}
