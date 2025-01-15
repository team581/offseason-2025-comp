package frc.robot.autos.auto_commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Trailblazer;
import frc.robot.robot_manager.RobotManager;

public class PushPartnerAuto extends BaseAuto {
  public PushPartnerAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Command getBlueAutoCommand() {
    return Commands.none();
  }

  @Override
  protected Command getRedAutoCommand() {
    return Commands.sequence(
        Commands.print("Red Push Partner Auto"),
        Commands.runOnce(
            () ->
                robotManager.localization.resetPose(
                    new Pose2d(9.47, 2.893, Rotation2d.fromDegrees(0.0)))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(9.47, 2.893, Rotation2d.fromDegrees(0.0))),
                new AutoPoint(new Pose2d(10.41, 2.892, Rotation2d.fromDegrees(0.0))),
                new AutoPoint(new Pose2d(11.146, 1.921, Rotation2d.fromDegrees(0.0))),
                new AutoPoint(new Pose2d(12.531, 2.757, Rotation2d.fromDegrees(60.61))))),
        Commands.print("score coral 1 (J)"),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(14.296, 1.669, Rotation2d.fromDegrees(146.97))),
                new AutoPoint(new Pose2d(15.872, 0.629, Rotation2d.fromDegrees(128.18))))),
        Commands.print("intake from S3"),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(14.532, 1.794, Rotation2d.fromDegrees(128.33))),
                new AutoPoint(new Pose2d(13.586, 2.757, Rotation2d.fromDegrees(121.86))))),
        Commands.print("score coral 2 (K)"),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(14.817, 1.794, Rotation2d.fromDegrees(133.48))),
                new AutoPoint(new Pose2d(16.295, 0.968, Rotation2d.fromDegrees(126.25))))),
        Commands.print("intake from S2"),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(14.939, 2.046, Rotation2d.fromDegrees(132.26))),
                new AutoPoint(new Pose2d(13.906, 2.893, Rotation2d.fromDegrees(119.57))))),
        Commands.print("score coral 3 (L)"),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(14.104, 3.111, Rotation2d.fromDegrees(119.57))))));
  }
}
