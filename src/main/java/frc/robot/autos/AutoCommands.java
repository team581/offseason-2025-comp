package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;

public class AutoCommands {
  private static Command followPathForAlliance(PathPlannerPath redPath, PathPlannerPath bluePath) {
    return Commands.either(
        AutoBuilder.followPath(redPath),
        AutoBuilder.followPath(bluePath),
        FmsSubsystem::isRedAlliance);
  }

  private final RobotCommands robotCommands;
  private final RobotManager robotManager;

  public AutoCommands(RobotCommands robotCommands, RobotManager robotManager) {
    this.robotCommands = robotCommands;
    this.robotManager = robotManager;
  }

  public Command doNothingCommand() {
    return Commands.none();
  }

}