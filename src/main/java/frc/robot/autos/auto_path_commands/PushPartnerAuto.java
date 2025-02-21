package frc.robot.autos.auto_path_commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
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
        Commands.runOnce(robotManager::rehomeRollRequest),
        // PUSH PARTNER
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(Points.AUTO_START_1.redPose),
                new AutoPoint(new Pose2d(10.31, 2.892, Rotation2d.kZero)))),

        // SCORE L4 ON J
        trailblazer
            .followSegment(
                new AutoSegment(
                    CONSTRAINTS,
                    new AutoPoint(new Pose2d(11.146, 1.921, Rotation2d.kZero)),
                    new AutoPoint(
                        new Pose2d(12.017, 2.207, Rotation2d.fromDegrees(60)),
                        autoCommands
                            .preloadCoralAfterRollHomed()
                            .andThen(autoCommands.l4WarmupCommand(ReefPipe.PIPE_J))),
                    new AutoPoint(robotManager.tagAlign::getUsedScoringPose)),
                false)
            .until(robotManager.tagAlign::isAligned),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(14.296, 1.669, Rotation2d.fromDegrees(146.97))),
                new AutoPoint(
                    Points.LEFT_CORAL_STATION.redPose, autoCommands.intakeStationWarmupCommand()))),
        autoCommands.intakeStationWithTimeoutCommand(),

        // SCORE L4 ON K
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_K)
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            CONSTRAINTS,
                            new AutoPoint(
                                new Pose2d(14.532, 1.794, Rotation2d.fromDegrees(128.33))),
                            new AutoPoint(robotManager.tagAlign::getUsedScoringPose)),
                        false)
                    .until(robotManager.tagAlign::isAligned),
                autoCommands.l4ScoreAndReleaseCommand()),

        // INTAKE STATION
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(14.817, 1.794, Rotation2d.fromDegrees(133.48))),
                new AutoPoint(
                    Points.LEFT_CORAL_STATION.redPose, autoCommands.intakeStationWarmupCommand()))),
        actions.intakeStationCommand(),

        // SCORE L4 ON L
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_L)
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            CONSTRAINTS,
                            new AutoPoint(
                                new Pose2d(14.939, 2.046, Rotation2d.fromDegrees(132.26))),
                            new AutoPoint(robotManager.tagAlign::getUsedScoringPose)),
                        false)
                    .until(robotManager.tagAlign::isAligned)),
        autoCommands.l4ScoreAndReleaseCommand());
  }
}
