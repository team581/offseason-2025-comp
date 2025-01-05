package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;

public class AutoCommands {
  private static Command followPathForAlliance(PathPlannerPath redPath, PathPlannerPath bluePath) {
    return Commands.either(
        AutoBuilder.followPath(redPath),
        AutoBuilder.followPath(bluePath),
        FmsSubsystem::isRedAlliance);
  }

  private final RobotCommands robotCommands;
  private final RobotManager robotManager;
  private static final boolean USE_DYNAMIC_AUTOS = true;

  public AutoCommands(RobotCommands robotCommands, RobotManager robotManager) {
    this.robotCommands = robotCommands;
    this.robotManager = robotManager;
  }

  public Command doNothingCommand() {
    return Commands.none();
  }

  private boolean hasNote() {
    if (!USE_DYNAMIC_AUTOS) {
      return true;
    }

    return robotManager.queuer.hasNote() || robotManager.getState() == RobotState.IDLE_WITH_GP;
  }

  public Command speakerShotWithTimeout() {
    return robotCommands.speakerCommand().withTimeout(3).withName("SpeakerShotWithTimeout");
  }

}
