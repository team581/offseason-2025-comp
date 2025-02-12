package frc.robot.autos.auto_path_commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
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
        // HOME AND PRELOAD
        actions.rehomeRollCommand(),
        autoCommands.preloadCoralCommand(),

        // SCORE L4 ON I
        trailblazer
            .followSegment(
                new AutoSegment(
                    CONSTRAINTS,
                    new AutoPoint(getRedStartingPose()),
                    new AutoPoint(
                        new Pose2d(11.785, 2.0, Rotation2d.fromDegrees(60)),
                        Commands.runOnce(
                            () -> {
                              AutoAlign.setAutoReefPipeOverride(ReefPipe.PIPE_I);
                              robotManager.highApproachRequest();
                            })),
                    new AutoPoint(ReefPipe.PIPE_I.getPose(ReefPipeLevel.L4))))
            .until(() -> robotManager.purpleAligned()),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(13.701, 1.795, Rotation2d.fromDegrees(135.88))),
                new AutoPoint(
                    new Pose2d(16.21, 0.9, Rotation2d.fromDegrees(127.71)),
                    Commands.runOnce(() -> robotManager.intakeStationRequest())))),
        autoCommands.intakeStationWithTimeoutCommand(),
        // SCORE L4 ON J
        trailblazer
            .followSegment(
                new AutoSegment(
                    CONSTRAINTS,
                    new AutoPoint(new Pose2d(13.872, 1.009, Rotation2d.fromDegrees(137.35))),
                    new AutoPoint(
                        new Pose2d(11.785, 1.4, Rotation2d.fromDegrees(60.0)),
                        Commands.runOnce(
                            () -> {
                              AutoAlign.setAutoReefPipeOverride(ReefPipe.PIPE_J);
                              robotManager.highApproachRequest();
                            })),
                    new AutoPoint(ReefPipe.PIPE_J.getPose(ReefPipeLevel.L4))))
            .until(() -> robotManager.purpleAligned()),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(13.872, 1.903, Rotation2d.fromDegrees(135.88))),
                new AutoPoint(
                    new Pose2d(16.21, 0.9, Rotation2d.fromDegrees(127.71)),
                    Commands.runOnce(() -> robotManager.intakeStationRequest())))),
        autoCommands.intakeStationWithTimeoutCommand(),

        // SCORE L4 ON K
        Commands.runOnce(
            () -> {
              AutoAlign.setAutoReefPipeOverride(ReefPipe.PIPE_K);
              robotManager.highApproachRequest();
            }),
        trailblazer
            .followSegment(
                new AutoSegment(
                    CONSTRAINTS,
                    new AutoPoint(new Pose2d(14.506, 1.903, Rotation2d.fromDegrees(133.277))),
                    new AutoPoint(
                        // REEF PIPE K
                        ReefPipe.PIPE_K.getPose(ReefPipeLevel.L4))))
            .until(() -> robotManager.purpleAligned()),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(14.506, 1.795, Rotation2d.fromDegrees(133.277))),
                new AutoPoint(
                    new Pose2d(16.21, 0.9, Rotation2d.fromDegrees(123.819)),
                    Commands.runOnce(() -> robotManager.intakeStationRequest())))),
        autoCommands.intakeStationWithTimeoutCommand(),

        // SCORE L4 ON L
        Commands.runOnce(
            () -> {
              AutoAlign.setAutoReefPipeOverride(ReefPipe.PIPE_L);
              robotManager.highApproachRequest();
            }),
        trailblazer
            .followSegment(
                new AutoSegment(
                    CONSTRAINTS,
                    new AutoPoint(new Pose2d(14.954, 1.971, Rotation2d.fromDegrees(134.931))),
                    // REEF PIPE L
                    new AutoPoint(ReefPipe.PIPE_L.getPose(ReefPipeLevel.L4))))
            .until(() -> robotManager.purpleAligned()),
        autoCommands.l4ScoreAndReleaseCommand());
  }
}
