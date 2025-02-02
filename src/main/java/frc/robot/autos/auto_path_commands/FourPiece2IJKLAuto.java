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

public class FourPiece2IJKLAuto extends BaseAuto {
  private static final AutoConstraintOptions CONSTRAINTS = new AutoConstraintOptions(1, 50, 4, 30);

  public FourPiece2IJKLAuto(RobotManager robotManager, Trailblazer trailblazer) {
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
        autoCommands.preloadCoralCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(10.289, 1.903, Rotation2d.fromDegrees(0.0))),
                new AutoPoint(
                    new Pose2d(11.171, 2.443, Rotation2d.fromDegrees(23.844)),
                    autoCommands.l4LineupCommand()),
                new AutoPoint(new Pose2d(12.212, 2.932, Rotation2d.fromDegrees(58.446))))),
        autoCommands.l4ScoreAndReleaseCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(13.301, 1.971, Rotation2d.fromDegrees(135.88))),
                new AutoPoint(new Pose2d(15.81, 0.6, Rotation2d.fromDegrees(127.71))))),
        autoCommands.intakeStationWithTimeoutCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(
                    new Pose2d(13.982, 1.637, Rotation2d.fromDegrees(135.878)),
                    autoCommands.l4LineupCommand()),
                // REEF PIPE J
                new AutoPoint(new Pose2d(12.497, 2.768, Rotation2d.fromDegrees(59.1))))),
        autoCommands.l4ScoreAndReleaseCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(13.872, 1.903, Rotation2d.fromDegrees(135.88))),
                new AutoPoint(new Pose2d(15.81, 0.6, Rotation2d.fromDegrees(127.71))))),
        autoCommands.intakeStationWithTimeoutCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(
                    new Pose2d(14.506, 1.903, Rotation2d.fromDegrees(0)),
                    autoCommands.l4LineupCommand()),
                new AutoPoint(
                    // REEF PIPE K
                    new Pose2d(13.593, 2.760, Rotation2d.fromDegrees(121.252))))),
        autoCommands.l4ScoreAndReleaseCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(14.506, 1.903, Rotation2d.fromDegrees(133.277))),
                new AutoPoint(new Pose2d(15.81, 0.6, Rotation2d.fromDegrees(123.819))))),
        autoCommands.intakeStationWithTimeoutCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(
                    new Pose2d(14.954, 1.971, Rotation2d.fromDegrees(134.931)),
                    autoCommands.l4LineupCommand()),
                // REEF PIPE L
                new AutoPoint(new Pose2d(13.877, 2.932, Rotation2d.fromDegrees(120.471))))),
        autoCommands.l4ScoreAndReleaseCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(13.877, 2.932, Rotation2d.fromDegrees(120.471))))));
  }
}
