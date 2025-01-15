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

public class Three3Piece2IJKAuto extends BaseAuto {
  public Three3Piece2IJKAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Command getBlueAutoCommand() {
    return Commands.none();
  }

  @Override
  protected Command getRedAutoCommand() {
    return Commands.sequence(
        Commands.print("Red Three Piece 2 IJK"),
        Commands.runOnce(
            () ->
                robotManager.localization.resetPose(
                    new Pose2d(10.380, 1.903, Rotation2d.fromDegrees(0.0)))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(10.380, 1.903, Rotation2d.fromDegrees(0.0))),
                new AutoPoint(new Pose2d(12.493, 2.774, Rotation2d.fromDegrees(58.45))))),
        Commands.print("score coral 1 (I)"),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(14.202, 1.568, Rotation2d.fromDegrees(135.88))),
                new AutoPoint(new Pose2d(15.686, 0.524, Rotation2d.fromDegrees(127.71))))),
        Commands.print("intake from S3"),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(14.66, 1.568, Rotation2d.fromDegrees(137.36))),
                new AutoPoint(new Pose2d(13.506, 2.774, Rotation2d.fromDegrees(116.76))))),
        Commands.print("score coral 2 (J)"),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(14.202, 1.568, Rotation2d.fromDegrees(131.81))),
                new AutoPoint(new Pose2d(15.686, 0.524, Rotation2d.fromDegrees(127.71))))),
        Commands.print("intake from S3"),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(13.983, 2.925, Rotation2d.fromDegrees(123.43))))),
        Commands.print("score coral 3 (K)"),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(14.104, 3.111, Rotation2d.fromDegrees(119.57))))));
  }
}
