package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import java.util.List;

public class AutoCommands {
  private final RobotCommands robotCommands;
  private final RobotManager robotManager;
  private final Subsystem[] requirements;

  public AutoCommands(RobotCommands robotCommands, RobotManager robotManager) {
    this.robotCommands = robotCommands;
    this.robotManager = robotManager;
    var requirementsList =
        List.of(robotManager.elevator, robotManager.wrist, robotManager.roll, robotManager.intake);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command doNothingCommand() {
    return Commands.none();
  }

  public Command l4LineupCommand() {
    return Commands.runOnce(robotManager::l4CoralLineupRequest, requirements);
  }

  public Command l4ScoreAndReleaseCommand() {
    return Commands.run(robotManager::l4coralPlaceAndReleaseRequest, requirements)
        .until(() -> robotManager.getState() == RobotState.IDLE_NO_GP)
        .withTimeout(4);
  }

  public Command intakeStationWaitUntilCommand() {
    return Commands.run(robotManager::intakeStationRequest, requirements)
        .until(() -> robotManager.getState() == RobotState.IDLE_CORAL)
        .withTimeout(4);
  }
}
