package frc.robot.autos.auto_path_commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.ReefSide;
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
  protected Pose2d getBlueStartingPose() {
    return Pose2d.kZero;
  }

  @Override
  protected Command getBlueAutoCommand() {
    return Commands.none();
  }

  @Override
  protected Pose2d getRedStartingPose() {
    return new Pose2d(10.289, 1.903, Rotation2d.kZero);
  }

  @Override
  protected Command getRedAutoCommand() {
    return Commands.sequence(
        actions.rehomeRollCommand(),
        autoCommands.preloadCoralCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(getRedStartingPose()),
                new AutoPoint(
                    new Pose2d(11.785, 2.622, Rotation2d.fromDegrees(58.446)),
                    Commands.runOnce(
                        () -> {
                          AutoAlign.setAutoReefSideOverride(ReefSide.SIDE_IJ);
                          robotManager.l4CoralLineupRequest();
                        })),
                // REEF PIPE I
                new AutoPoint(new Pose2d(12.246, 2.952, Rotation2d.fromDegrees(58.446))))),
        autoCommands.l4ScoreAndReleaseCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(
                    new Pose2d(13.701, 1.795, Rotation2d.fromDegrees(135.88)),
                    Commands.runOnce(() -> robotManager.intakeStationRequest())),
                new AutoPoint(new Pose2d(15.81, 0.6, Rotation2d.fromDegrees(127.71))))),
        autoCommands.intakeStationWithTimeoutCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(
                    new Pose2d(13.982, 1.637, Rotation2d.fromDegrees(135.878)),
                    Commands.runOnce(
                        () -> {
                          AutoAlign.setAutoReefSideOverride(ReefSide.SIDE_IJ);
                          robotManager.l4CoralLineupRequest();
                        })),
                // REEF PIPE J
                new AutoPoint(new Pose2d(12.497, 2.668, Rotation2d.fromDegrees(59.1))))),
        autoCommands.l4ScoreAndReleaseCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(
                    new Pose2d(13.872, 1.903, Rotation2d.fromDegrees(135.88)),
                    Commands.runOnce(() -> robotManager.intakeStationRequest())),
                new AutoPoint(new Pose2d(15.81, 0.6, Rotation2d.fromDegrees(127.71))))),
        autoCommands.intakeStationWithTimeoutCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(
                    new Pose2d(14.506, 1.903, Rotation2d.kZero),
                    Commands.runOnce(
                        () -> {
                          AutoAlign.setAutoReefSideOverride(ReefSide.SIDE_KL);
                          robotManager.l4CoralLineupRequest();
                        })),
                new AutoPoint(
                    // REEF PIPE K
                    new Pose2d(13.566, 2.812, Rotation2d.fromDegrees(121.252))))),
        autoCommands.l4ScoreAndReleaseCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(
                    new Pose2d(14.506, 1.795, Rotation2d.fromDegrees(133.277)),
                    Commands.runOnce(() -> robotManager.intakeStationRequest())),
                new AutoPoint(new Pose2d(15.81, 0.6, Rotation2d.fromDegrees(123.819))))),
        autoCommands.intakeStationWithTimeoutCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(
                    new Pose2d(14.954, 1.971, Rotation2d.fromDegrees(134.931)),
                    Commands.runOnce(
                        () -> {
                          AutoAlign.setAutoReefSideOverride(ReefSide.SIDE_KL);
                          robotManager.l4CoralLineupRequest();
                        })),
                // REEF PIPE L
                new AutoPoint(new Pose2d(13.922, 2.842, Rotation2d.fromDegrees(120.471))))),
        autoCommands.l4ScoreAndReleaseCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(13.922, 2.842, Rotation2d.fromDegrees(120.471))))));
  }
}
