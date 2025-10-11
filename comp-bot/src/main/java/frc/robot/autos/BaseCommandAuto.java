package frc.robot.autos;

import com.team581.GlobalConfig;
import com.team581.autos.AbstractCommandAuto;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.TrailblazerPathLogger;
import dev.doglog.DogLog;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;

public abstract class BaseCommandAuto extends AbstractCommandAuto {
  protected final RobotManager robotManager;
  protected final Trailblazer trailblazer;
  protected final RobotCommands actions;
  protected final AutoCommands autoCommands;
  protected final AutoBlocks blocks;

  protected BaseCommandAuto(RobotManager robotManager, Trailblazer trailblazer) {
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;
    actions = new RobotCommands(robotManager);
    autoCommands = new AutoCommands(actions, robotManager);
    blocks = new AutoBlocks(trailblazer, robotManager, autoCommands);
  }

  @Override
  protected Command createFullAutoCommand() {
    TrailblazerPathLogger.markAuto(this);
    // We continuously reset the pose anyway, but doing it here should be fine
    // It's basically free as long as we aren't updating the IMU
    return timing
        .time(
            "TotalTime",
            // TODO: Seems like this doesn't run or runs incorrectly in sim
            Commands.runOnce(() -> robotManager.localization.resetPose(getStartingPose())),
            createAutoCommand())
        .finallyDo(
            interrupted -> {
              // Stop driving once the auto finishes
              robotManager.swerve.setFieldRelativeAutoSpeeds(new ChassisSpeeds());

              // Check if we are enabled, since auto commands are cancelled during disable
              if (interrupted && DriverStation.isAutonomousEnabled()) {
                DogLog.logFault("Auto command interrupted outside teleop");

                if (GlobalConfig.IS_DEVELOPMENT) {
                  throw new IllegalStateException(
                      "The auto command was interrupted while still in auto mode, is there a command requirements conflict?");
                }
              }
            })
        .withName(name() + "Command");
  }
}
