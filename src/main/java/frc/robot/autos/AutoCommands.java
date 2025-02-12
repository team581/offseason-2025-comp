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
    return Commands.none().withName("DoNothingCommand");
  }

  public Command l4LineupCommand() {
    return Commands.runOnce(robotManager::l4CoralLineupRequest, requirements)
        .withName("L4LineupCommand");
  }

  public Command l4ScoreAndReleaseCommand() {
    return Commands.runOnce(robotManager::l4coralPlaceAndReleaseRequest, requirements)
        .andThen(robotManager.waitForState(RobotState.IDLE_NO_GP).withTimeout(1))
        .andThen(robotCommands.stowCommand().withTimeout(1))
        .withName("L4ScoreAndReleaseCommand");
  }

  public Command intakeStationWithTimeoutCommand() {
    return Commands.runOnce(robotManager::intakeStationRequest, requirements)
    .andThen(robotManager.waitForStates(RobotState.SMART_STOW_1, RobotState.SMART_STOW_2, RobotState.IDLE_CORAL))
    .withTimeout(3)
    .withName("IntakeStationWithTimeoutCommand");
}

  public Command preloadCoralCommand() {
    return Commands.runOnce(robotManager::preloadCoralRequest, requirements)
        .withName("PreloadCoralCommand");
  }
}
