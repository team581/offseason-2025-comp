package frc.robot.autos;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;

public class AutoCommands {
  private final RobotManager robotManager;

  public AutoCommands(RobotCommands robotCommands, RobotManager robotManager) {
    this.robotManager = robotManager;
  }

  public Command stowCommand() {
    return Commands.runOnce(robotManager::stowRequest).withName("StowRequestCommand");
  }

  public Command preloadCoralCommand() {
    return Commands.runOnce(robotManager::preloadCoralRequest).withName("PreloadCommand");
  }

  public boolean alignedForScore() {
    return robotManager.autoAlign.isAligned()
        && robotManager.imu.isFlatDebounced()
        && robotManager.elevator.atGoal()
        && robotManager.wrist.atGoal();
  }

  public Command intakeCoralCommand() {
    return Commands.runOnce(robotManager::intakeCoralRequest).withName("IntakeCoralCommand");
  }

  public Command intakeAlgaeL2Command() {
    return Commands.runOnce(robotManager::intakeL2AlgaeRequest).withName("IntakeAlgaeL2Command");
  }

  public Command intakeAlgaeL3Command() {
    return Commands.runOnce(robotManager::intakeL3AlgaeRequest).withName("IntakeAlgaeL3Command");
  }

  public Command waitForAlignedForScore() {
    if (RobotBase.isSimulation()) {
      return Commands.waitSeconds(1.0);
    }
    return Commands.waitUntil(this::alignedForScore)
        .withTimeout(5)
        .withName("WaitForAlignedForScore");
  }

  public Command netWaitCommand() {
    return Commands.runOnce(robotManager::netWaitRequest).withName("NetWaitCommand");
  }

  public Command netReleaseCommand() {
    return Commands.runOnce(robotManager::netReleaseRequest).withName("NetReleaseCommand");
  }

  public Command waitForReleaseCommand() {
    return robotManager
        .waitForStates(
            RobotState.CORAL_L1_RELEASE,
            RobotState.ALGAE_NET_RELEASE,
            RobotState.ALGAE_PROCESSOR_RELEASE)
        .withName("WaitForReleaseCommand");
  }

  public Command l1ApproachCommand(ReefPipe pipe) {
    return Commands.runOnce(
            () -> {
              pipe.getPose(ReefPipeLevel.L1);
              robotManager.l1ApproachRequest();
            })
        .withName("L1ApproachCommand");
  }

  public Command l1LineupCommand(ReefPipe pipe) {
    return Commands.runOnce(
            () -> {
              pipe.getPose(ReefPipeLevel.L1);
              robotManager.l1lineupRequest();
            })
        .withName("L1LineupCommand");
  }

  public Command l1ReleaseCommand(ReefPipe pipe) {
    return Commands.runOnce(
            () -> {
              pipe.getPose(ReefPipeLevel.L1);
              robotManager.l1ReleaseRequest();
            })
        .withName("L1ReleaseCommand");
  }
}
