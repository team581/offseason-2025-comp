package frc.robot.autos.auto_path_commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Trailblazer;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.robot_manager.RobotManager;

public class ThreePiece1DEC extends BaseAuto {
  private static final AutoConstraintOptions CONSTRAINTS =
      new AutoConstraintOptions(false, 4.75, 71.5, 8.5, 35.2);

  public ThreePiece1DEC(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Command getBlueAutoCommand() {
    return Commands.none();
  }

  @Override
  protected Command getRedAutoCommand() {
    return Commands.sequence(
        Commands.print(""),
        Commands.runOnce(
            () ->
                robotManager.localization.resetPose(
                    new Pose2d(10.289, 3.047, Rotation2d.fromDegrees(0.0)))),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(11.075, 4.443, Rotation2d.fromDegrees(0.0))),
                new AutoPoint(new Pose2d(12.493, 5.085, Rotation2d.fromDegrees(-58.45))))),
        Commands.print("score coral (D)"),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(14.202, 6.482, Rotation2d.fromDegrees(-135.88))),
                new AutoPoint(new Pose2d(15.902, 7.497, Rotation2d.fromDegrees(-127.71))))),
        Commands.print("intake"),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(14.66, 6.482, Rotation2d.fromDegrees(-137.36))),
                new AutoPoint(new Pose2d(13.562, 5.180, Rotation2d.fromDegrees(-119.50))))),
        Commands.print("score coral (E)"),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(14.66, 6.482, Rotation2d.fromDegrees(-131.81))),
                new AutoPoint(new Pose2d(15.902, 7.497, Rotation2d.fromDegrees(-127.71))))),
        Commands.print("intake"),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(15.014, 6.312, Rotation2d.fromDegrees(-130.68))),
                new AutoPoint(new Pose2d(13.918, 4.933, Rotation2d.fromDegrees(-130.06))))),
        Commands.print("score coral (C)"),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(14.102, 4.789, Rotation2d.fromDegrees(-123.427))))));
  }
}
