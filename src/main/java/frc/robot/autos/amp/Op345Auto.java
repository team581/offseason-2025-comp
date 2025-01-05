package frc.robot.autos.amp;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.trailblazer.AutoPoint;
import frc.robot.autos.trailblazer.AutoSegment;
import frc.robot.autos.trailblazer.Trailblazer;
import frc.robot.robot_manager.RobotManager;

public class Op345Auto extends BaseAuto {
  public Op345Auto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Command getBlueAutoCommand() {
    return Commands.none();
  }

  @Override
  protected Command getRedAutoCommand() {
    return Commands.sequence(
        Commands.print("example command on auto start"),
        Commands.runOnce(
            () ->
                robotManager.localization.resetPose(
                    new Pose2d(15.78, 6.67, new Rotation2d(-58.39)))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(15.78, 6.67, new Rotation2d(-58.39))),
                new AutoPoint(
                    new Pose2d(14.676, 6.767, new Rotation2d(-26.85)),
                    Commands.sequence(
                        autoCommands.speakerShotWithTimeout(), actions.intakeAssistCommand())),
                new AutoPoint(new Pose2d(13.68, 6.99, new Rotation2d(0.0))),
                new AutoPoint(
                    new Pose2d(12.64, 7.06, new Rotation2d(-21.67)),
                    Commands.sequence(
                        autoCommands.speakerShotWithTimeout(), actions.intakeAssistCommand())),
                new AutoPoint(new Pose2d(8.52, 7.46, new Rotation2d(0.0))),
                new AutoPoint(
                    new Pose2d(12.64, 7.06, new Rotation2d(-21.67)),
                    autoCommands.speakerShotWithTimeout()),
                new AutoPoint(
                    new Pose2d(11.03, 6.88, new Rotation2d(30.17)), actions.intakeAssistCommand()),
                new AutoPoint(
                    new Pose2d(10.14, 6.41, new Rotation2d(25.43)), actions.intakeAssistCommand()),
                new AutoPoint(
                    new Pose2d(8.32, 5.79, new Rotation2d(24.67)), actions.intakeAssistCommand()),
                new AutoPoint(
                    new Pose2d(12.64, 7.06, new Rotation2d(-21.67)),
                    autoCommands.speakerShotWithTimeout()))));
  }
}
