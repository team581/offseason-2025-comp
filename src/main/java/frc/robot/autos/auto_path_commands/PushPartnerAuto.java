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

public class PushPartnerAuto extends BaseAuto {
  private static final AutoConstraintOptions CONSTRAINTS =
      new AutoConstraintOptions(4.75, 71.5, 8.5, 35.2);

  public PushPartnerAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getBlueStartingPose() {
    return Pose2d.kZero;
  }

  @Override
  protected Command getBlueAutoCommand() {
    return Commands.none();
  }

  @Override
  protected Pose2d getRedStartingPose() {
    return new Pose2d(9.57, 2.893, Rotation2d.kZero);
  }

  @Override
  protected Command getRedAutoCommand() {
    return Commands.sequence(
        Commands.print("Red Push Partner Auto"),
        actions.rehomeRollCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(getRedStartingPose()),
                new AutoPoint(new Pose2d(10.31, 2.892, Rotation2d.kZero)))),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(11.146, 1.921, Rotation2d.kZero)),
                new AutoPoint(new Pose2d(12.529, 2.892, Rotation2d.fromDegrees(56.63))))),
        Commands.sequence(autoCommands.l4LineupCommand(), actions.confirmScoreCommand()),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(14.296, 1.669, Rotation2d.fromDegrees(146.97))),
                new AutoPoint(new Pose2d(15.862, 0.552, Rotation2d.fromDegrees(128.18))))),
        actions.intakeStationCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(14.532, 1.794, Rotation2d.fromDegrees(128.33))),
                new AutoPoint(new Pose2d(13.572, 2.892, Rotation2d.fromDegrees(121.86))))),
        Commands.sequence(autoCommands.l4LineupCommand(), actions.confirmScoreCommand()),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(14.817, 1.794, Rotation2d.fromDegrees(133.48))),
                new AutoPoint(new Pose2d(15.862, 0.552, Rotation2d.fromDegrees(126.25))))),
        actions.intakeStationCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(14.939, 2.046, Rotation2d.fromDegrees(132.26))),
                new AutoPoint(new Pose2d(13.868, 3.005, Rotation2d.fromDegrees(123.81))))),
        Commands.sequence(autoCommands.l4LineupCommand(), actions.confirmScoreCommand()),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(14.052, 3.161, Rotation2d.fromDegrees(123.81))))));
  }
}
