package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
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

  public Command resetPoseIfNeeded(Pose2d pose) {
    return Commands.runOnce(
        () -> {
          if (!robotManager.vision.hasSeenTag()) {
            robotManager.localization.resetPose(pose);
          }
        });
  }
}
