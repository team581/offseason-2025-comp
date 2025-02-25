package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.auto_align.ReefPipe;
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

  public Command l4ScoreAndReleaseCommand() {
    return Commands.runOnce(robotManager::l4coralPlaceAndReleaseRequest, requirements)
        .andThen(
            robotManager
                .waitForStates(
                    RobotState.CORAL_DISPLACED_L4_4_RELEASE, RobotState.CORAL_CENTERED_L4_4_RELEASE)
                .withTimeout(1))
        .withName("L4ScoreAndReleaseCommand");
  }

  @Deprecated
  public Command intakeStationWithTimeoutCommand() {
    return Commands.runOnce(robotManager::intakeStationRequest, requirements)
        .andThen(
            robotManager.waitForStates(
                RobotState.SMART_STOW_1, RobotState.SMART_STOW_2, RobotState.IDLE_CORAL))
        .withTimeout(2)
        .withName("IntakeStationWithTimeoutCommand");
  }

  public Command l4WarmupCommand(ReefPipe pipe) {
    return Commands.waitUntil(
            () ->
                robotManager.getState() != RobotState.SMART_STOW_1
                    && robotManager.getState() != RobotState.SMART_STOW_2)
        .andThen(
            Commands.runOnce(
                () -> {
                  robotManager.autoAlign.setAutoReefPipeOverride(pipe);
                  robotManager.l4CoralApproachRequest();
                }));
  }

  public Command intakeStationWarmupCommand() {
    return Commands.runOnce(robotManager::intakeStationRequest);
  }

  public Command preloadCoralAfterRollHomed() {
    return robotManager
        .waitForRollHomedCommand()
        .andThen(Commands.runOnce(robotManager::preloadCoralRequest));
  }

  public boolean isSmartStowing() {
    return robotManager.getState() == RobotState.SMART_STOW_1
        || robotManager.getState() == RobotState.SMART_STOW_2;
  }

  public Command stowRequest() {
    return Commands.runOnce(robotManager::stowRequest);
  }

  public boolean alignedForScore() {
    return robotManager.autoAlign.isTagAlignedDebounced() && robotManager.imu.isFlatDebounced();
  }
}
