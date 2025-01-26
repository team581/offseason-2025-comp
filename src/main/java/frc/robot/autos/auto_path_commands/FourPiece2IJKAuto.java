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

public class FourPiece2IJKAuto extends BaseAuto {
  private static final AutoConstraintOptions CONSTRAINTS = new AutoConstraintOptions(1, 50, 4, 30);

  public FourPiece2IJKAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Command getBlueAutoCommand() {
    return Commands.none();
  }

  @Override
  protected Command getRedAutoCommand() {
    return Commands.sequence(
        Commands.print("Red Three Piece 2 IJK Auto"),
        Commands.runOnce(
            () ->
                robotManager.localization.resetPose(
                    new Pose2d(10.289, 1.903, Rotation2d.fromDegrees(0.0)))),
        actions.rehomeRollCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(10.289, 1.903, Rotation2d.fromDegrees(0.0))),
                new AutoPoint(new Pose2d(12.493, 2.965, Rotation2d.fromDegrees(58.45))))),
        Commands.sequence(autoCommands.l4LineupCommand(), actions.confirmScoreCommand()),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(14.202, 1.568, Rotation2d.fromDegrees(135.88))),
                new AutoPoint(new Pose2d(15.902, 0.553, Rotation2d.fromDegrees(127.71))))),
        actions.intakeStationCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(14.66, 1.568, Rotation2d.fromDegrees(137.36))),
                new AutoPoint(new Pose2d(13.562, 2.87, Rotation2d.fromDegrees(119.50))))),
        Commands.sequence(autoCommands.l4LineupCommand(), actions.confirmScoreCommand()),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(14.391, 1.568, Rotation2d.fromDegrees(135.88))),
                new AutoPoint(new Pose2d(15.802, 0.553, Rotation2d.fromDegrees(127.71))))),
        actions.intakeStationCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(13.918, 3.117, Rotation2d.fromDegrees(123.427))))),
        Commands.sequence(autoCommands.l4LineupCommand(), actions.confirmScoreCommand()),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(15.093, 1.971, Rotation2d.fromDegrees(133.277))),
                new AutoPoint(new Pose2d(16.292, 0.842, Rotation2d.fromDegrees(123.819))))),
        actions.intakeStationCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(15.295, 2.435, Rotation2d.fromDegrees(115.844))),
                new AutoPoint(new Pose2d(14.391, 3.81, Rotation2d.fromDegrees(176.077))))),
        Commands.sequence(autoCommands.l4LineupCommand(), actions.confirmScoreCommand()),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(14.391, 3.81, Rotation2d.fromDegrees(176.077))))));
  }
}
