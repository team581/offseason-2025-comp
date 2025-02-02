package frc.robot.autos;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;

public abstract class BaseAuto {
  protected final RobotManager robotManager;
  protected final Trailblazer trailblazer;
  protected final RobotCommands actions;
  protected final AutoCommands autoCommands;

  protected BaseAuto(RobotManager robotManager, Trailblazer trailblazer) {
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;
    this.actions = new RobotCommands(robotManager);
    this.autoCommands = new AutoCommands(actions, robotManager);
  }

  protected abstract Command getRedAutoCommand();

  protected abstract Command getBlueAutoCommand();

  public Command getAutoCommand() {
    var className = this.getClass().getSimpleName();
    className = className.substring(className.lastIndexOf('.') + 1);

    return Commands.either(
            getRedAutoCommand().withName(className + "RedCommand"),
            getBlueAutoCommand().withName(className + "BlueCommand"),
            FmsSubsystem::isRedAlliance)
        .withName(className + "Command")
        .finallyDo(
            interrupted -> {
              if (interrupted && DriverStation.isAutonomous()) {
                DogLog.logFault("Auto command interrupted outside teleop");

                if (RobotConfig.IS_DEVELOPMENT) {
                  throw new RuntimeException(
                      "The auto command was interrupted while still in auto mode, is there a command requirements conflict?");
                }
              }
            });
  }
}
