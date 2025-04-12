package frc.robot.autos;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import frc.robot.robot_manager.ground_manager.GroundState;
import java.util.List;

public class AutoCommands {
  private final RobotCommands robotCommands;
  private final RobotManager robotManager;
  private final Subsystem[] requirements;

  public AutoCommands(RobotCommands robotCommands, RobotManager robotManager) {
    this.robotCommands = robotCommands;
    this.robotManager = robotManager;
    var requirementsList = List.of(robotManager.elevator, robotManager.arm, robotManager.claw);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command stowRequest() {
    return Commands.runOnce(robotManager::stowRequest);
  }

  public Command preloadCoralCommand() {
    return Commands.runOnce(robotManager::preloadCoralRequest);
  }

  public boolean alignedForScore() {
    return robotManager.autoAlign.isTagAlignedDebounced()
        && robotManager.imu.isFlatDebounced()
        && robotManager.elevator.atGoal()
        && robotManager.arm.atGoal();
  }

  public Command intakeCoralHorizontalCommand() {
    return Commands.runOnce(robotManager.groundManager::intakeRequest);
  }

  public Command lollipopApproachCommand() {
    return Commands.runOnce(robotManager::lollipopIntakeApproachRequest);
  }

  public Command intakeLollipopCommand() {
    return Commands.runOnce(robotManager::lollipopIntakeGrabRequest);
  }

  public Command homeDeployCommand() {
    return Commands.runOnce(robotManager.groundManager::rehomeDeployRequest);
  }

  public Command waitForIntakeDone() {
    if (RobotBase.isSimulation()) {
      return Commands.waitSeconds(1.0);
    }
    return robotManager
        .groundManager
        .waitForStates(
            GroundState.IDLE_CORAL, GroundState.HANDOFF_WAIT, GroundState.HANDOFF_RELEASE)
        .withTimeout(4);
  }

  public Command waitForLollipopIntakeDone() {
    return robotManager
        .waitForStates(RobotState.CORAL_INTAKE_LOLLIPOP_PUSH, RobotState.CLAW_CORAL)
        .withTimeout(1.5);
  }

  public Command waitForElevatorAndArmNearGoal() {
    return Commands.waitUntil(
        () -> robotManager.elevator.nearGoal() && robotManager.arm.nearGoal());
  }

  public Command l4ApproachCommand(ReefPipe pipe, RobotScoringSide scoringSide) {
    return Commands.runOnce(
        () -> {
          robotManager.autoAlign.setAutoReefPipeOverride(pipe);
          if (scoringSide == RobotScoringSide.LEFT) {
            robotManager.l4CoralLeftAutoApproachRequest();
          } else {
            robotManager.l4CoralRightAutoApproachRequest();
          }
        });
  }

  public Command l3ApproachCommand(RobotScoringSide scoringSide) {
    return Commands.runOnce(
        () -> {
          if (scoringSide == RobotScoringSide.LEFT) {
            robotManager.l3CoralLeftAutoApproachRequest();
          } else {
            robotManager.l3CoralRightAutoApproachRequest();
          }
        });
  }

  public Command l2ApproachCommand(RobotScoringSide scoringSide) {
    return Commands.runOnce(
        () -> {
          if (scoringSide == RobotScoringSide.LEFT) {
            robotManager.l2CoralLeftAutoApproachRequest();
          } else {
            robotManager.l2CoralRightAutoApproachRequest();
          }
        });
  }

  public Command l4LeftReleaseCommand(ReefPipe pipe, RobotScoringSide scoringSide) {
    return Commands.runOnce(
        () -> {
          robotManager.autoAlign.setAutoReefPipeOverride(pipe);
          robotManager.l4CoralLeftReleaseRequest();
        });
  }

  public Command l3LeftReleaseCommand() {
    return Commands.runOnce(robotManager::l3CoralLeftReleaseRequest);
  }

  public Command moveToStartingPositionCommand() {
    return Commands.runOnce(robotManager::startingPositionRequest);
  }

  public Command waitForReleaseCommand() {
    return robotManager.waitForStates(
        RobotState.CORAL_L2_LEFT_RELEASE,
        RobotState.CORAL_L4_LEFT_RELEASE,
        RobotState.CORAL_L3_LEFT_RELEASE,
        RobotState.CORAL_L2_RIGHT_RELEASE,
        RobotState.CORAL_L3_RIGHT_RELEASE,
        RobotState.CORAL_L4_RIGHT_RELEASE);
  }

  public Command waitForAlignedForScore() {
    if (RobotBase.isSimulation()) {
      return Commands.waitSeconds(1.0);
    }
    return Commands.waitUntil(this::alignedForScore).withTimeout(5);
  }

  public Command groundIntakeToL4Command() {
    return Commands.runOnce(
        () -> {
          robotManager.groundManager.intakeThenHandoffRequest();
          robotManager.l4CoralApproachRequest();
        });
  }
}
