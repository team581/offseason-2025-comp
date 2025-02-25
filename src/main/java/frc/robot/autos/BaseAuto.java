package frc.robot.autos;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
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
  private final String autoName;
  private final Command autoCommand;

  protected BaseAuto(RobotManager robotManager, Trailblazer trailblazer) {
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;
    this.actions = new RobotCommands(robotManager);
    this.autoCommands = new AutoCommands(actions, robotManager);

    var className = this.getClass().getSimpleName();
    autoName = className.substring(className.lastIndexOf('.') + 1);
    autoCommand = createAutoCommand();
  }

  protected abstract Pose2d getRedStartingPose();

  protected abstract Pose2d getBlueStartingPose();

  protected abstract Command getRedAutoCommand();

  protected abstract Command getBlueAutoCommand();

  /** Returns the name of this auto. */
  public String name() {
    return autoName;
  }

  public Command getAutoCommand() {
    return autoCommand;
  }

  private Command createAutoCommand() {
    // We continuously reset the pose anyway, but doing it here should be fine
    // It's basically free as long as we aren't updating the IMU
    return Commands.either(
            Commands.runOnce(() -> robotManager.localization.resetPose(getRedStartingPose()))
                .andThen(getRedAutoCommand())
                .withName(autoName + "RedCommand"),
            Commands.runOnce(() -> robotManager.localization.resetPose(getBlueStartingPose()))
                .andThen(getBlueAutoCommand())
                .withName(autoName + "BlueCommand"),
            FmsSubsystem::isRedAlliance)
        .finallyDo(
            interrupted -> {
              // Check if we are enabled, since auto commands are cancelled during disable
              if (interrupted && DriverStation.isAutonomousEnabled()) {
                DogLog.logFault("Auto command interrupted outside teleop");

                if (RobotConfig.IS_DEVELOPMENT) {
                  throw new RuntimeException(
                      "The auto command was interrupted while still in auto mode, is there a command requirements conflict?");
                }
              }
            })
        .withName(autoName + "Command");
  }
}
