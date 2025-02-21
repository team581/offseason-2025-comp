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
import frc.robot.robot_manager.RobotState;

public class FourPiece2IJKLAuto extends BaseAuto {
  private static final AutoConstraintOptions INTAKING_CONSTRAINTS =
      new AutoConstraintOptions(3.5, 57, 4, 30);
  private static final AutoConstraintOptions SCORING_CONSTRAINTS =
      new AutoConstraintOptions(2, 57, 4, 30);

  public FourPiece2IJKLAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getBlueStartingPose() {
    return Points.AUTO_START_2.bluePose;
  }

  @Override
  protected Command getBlueAutoCommand() {
    return Commands.sequence(
        Commands.runOnce(robotManager::rehomeRollRequest),
        // SCORE L4 ON I
        trailblazer
            .followSegment(
                new AutoSegment(
                    SCORING_CONSTRAINTS,
                    new AutoPoint(Points.AUTO_START_2.bluePose, INTAKING_CONSTRAINTS),
                    new AutoPoint(
                            new Pose2d(11.785, 2.0, Rotation2d.fromDegrees(60)),
                            autoCommands
                                .preloadCoralAfterRollHomed()
                                .andThen(autoCommands.l4WarmupCommand(ReefPipe.PIPE_I)))
                        .pathflipped(),
                    new AutoPoint(robotManager.autoAlign::getUsedScoringPose)),
                false)
            .until(robotManager.autoAlign::isTagAlignedDebounced),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer.followSegment(
            new AutoSegment(
                INTAKING_CONSTRAINTS,
                new AutoPoint(new Pose2d(11.960, 1.903, Rotation2d.fromDegrees(60))).pathflipped(),
                new AutoPoint(
                        new Pose2d(13.636, 1.439, Rotation2d.fromDegrees(135.88)),
                        Commands.runOnce(robotManager::stowRequest))
                    .pathflipped(),
                new AutoPoint(
                    Points.LEFT_CORAL_STATION.bluePose,
                    autoCommands.intakeStationWarmupCommand()))),
        autoCommands.intakeStationWithTimeoutCommand(),

        // SCORE L4 ON J
        trailblazer
            .followSegment(
                new AutoSegment(
                    SCORING_CONSTRAINTS,
                    new AutoPoint(new Pose2d(14.103, 1.0, Rotation2d.fromDegrees(90.5)), INTAKING_CONSTRAINTS)
                        .pathflipped(),
                    new AutoPoint(
                            new Pose2d(12.246, 1.244, Rotation2d.fromDegrees(60.0)),
                            autoCommands.l4WarmupCommand(ReefPipe.PIPE_J))
                        .pathflipped(),
                    new AutoPoint(robotManager.autoAlign::getUsedScoringPose)),
                false)
            .until(robotManager.autoAlign::isTagAlignedDebounced),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer.followSegment(
            new AutoSegment(
                INTAKING_CONSTRAINTS,
                new AutoPoint(new Pose2d(12.246, 2.087, Rotation2d.fromDegrees(60))).pathflipped(),
                new AutoPoint(
                        new Pose2d(13.842, 1.582, Rotation2d.fromDegrees(135.88)),
                        Commands.runOnce(robotManager::stowRequest))
                    .pathflipped(),
                new AutoPoint(
                    Points.LEFT_CORAL_STATION.bluePose,
                    autoCommands.intakeStationWarmupCommand()))),
        autoCommands.intakeStationWithTimeoutCommand(),

        // SCORE L4 ON K
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_K)
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            SCORING_CONSTRAINTS,
                            new AutoPoint(
                                    new Pose2d(14.506, 1.903, Rotation2d.fromDegrees(133.277)))
                                .pathflipped(),
                            // REEF PIPE K
                            new AutoPoint(robotManager.autoAlign::getUsedScoringPose)),
                        false)
                    .until(robotManager.autoAlign::isTagAlignedDebounced)),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer.followSegment(
            new AutoSegment(
                INTAKING_CONSTRAINTS,
                new AutoPoint(new Pose2d(14.284, 2.087, Rotation2d.fromDegrees(133.277)))
                    .pathflipped(),
                new AutoPoint(
                        new Pose2d(15.083, 1.439, Rotation2d.fromDegrees(133.277)),
                        Commands.runOnce(robotManager::stowRequest))
                    .pathflipped(),
                new AutoPoint(
                    Points.LEFT_CORAL_STATION.bluePose,
                    autoCommands.intakeStationWarmupCommand()))),
        autoCommands.intakeStationWithTimeoutCommand(),

        // SCORE L4 ON L
        robotManager
            .waitForStates(RobotState.IDLE_CORAL, RobotState.IDLE_NO_GP)
            .andThen(autoCommands.l4WarmupCommand(ReefPipe.PIPE_L))
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            SCORING_CONSTRAINTS,
                            new AutoPoint(
                                    new Pose2d(14.954, 1.971, Rotation2d.fromDegrees(134.931)))
                                .pathflipped(),
                            // REEF PIPE L
                            new AutoPoint(robotManager.autoAlign::getUsedScoringPose)),
                        false)
                    .until(robotManager.autoAlign::isTagAlignedDebounced)),
        autoCommands.l4ScoreAndReleaseCommand(),
        trailblazer.followSegment(
            new AutoSegment(
              SCORING_CONSTRAINTS,
                new AutoPoint(new Pose2d(13.998, 2.812, Rotation2d.fromDegrees(134.931)))
                    .pathflipped(),
                new AutoPoint(
                        new Pose2d(14.284, 2.435, Rotation2d.fromDegrees(134.931)),
                        Commands.runOnce(robotManager::stowRequest))
                    .pathflipped())));
  }

  @Override
  protected Pose2d getRedStartingPose() {
    return Points.AUTO_START_2.redPose;
  }

  @Override
  protected Command getRedAutoCommand() {
    return Commands.sequence(
        Commands.runOnce(robotManager::rehomeRollRequest),
        // SCORE L4 ON I
        trailblazer
            .followSegment(
                new AutoSegment(
                    SCORING_CONSTRAINTS,
                    new AutoPoint(Points.AUTO_START_2.redPose),
                    new AutoPoint(
                        new Pose2d(11.785, 2.0, Rotation2d.fromDegrees(60)),
                        autoCommands
                            .preloadCoralAfterRollHomed()
                            .andThen(autoCommands.l4WarmupCommand(ReefPipe.PIPE_I))),
                    new AutoPoint(robotManager.autoAlign::getUsedScoringPose)),
                false)
            .until(robotManager.autoAlign::isTagAlignedDebounced),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer.followSegment(
            new AutoSegment(
                INTAKING_CONSTRAINTS,
                new AutoPoint(new Pose2d(11.960, 1.903, Rotation2d.fromDegrees(60))),
                new AutoPoint(
                    new Pose2d(13.636, 1.439, Rotation2d.fromDegrees(135.88)),
                    Commands.runOnce(robotManager::stowRequest)),
                new AutoPoint(
                    Points.LEFT_CORAL_STATION.redPose, autoCommands.intakeStationWarmupCommand()))),
        autoCommands.intakeStationWithTimeoutCommand(),

        // SCORE L4 ON J
        trailblazer
            .followSegment(
                new AutoSegment(
                    SCORING_CONSTRAINTS,
                    new AutoPoint(new Pose2d(14.103, 1.0, Rotation2d.fromDegrees(90.5))),
                    new AutoPoint(
                        new Pose2d(12.246, 1.244, Rotation2d.fromDegrees(60.0)),
                        autoCommands.l4WarmupCommand(ReefPipe.PIPE_J)),
                    new AutoPoint(robotManager.autoAlign::getUsedScoringPose)),
                false)
            .until(robotManager.autoAlign::isTagAlignedDebounced),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        Commands.waitSeconds(0.5)
            .andThen(robotManager::stowRequest)
            .alongWith(
                trailblazer.followSegment(
                    new AutoSegment(
                        INTAKING_CONSTRAINTS,
                        new AutoPoint(new Pose2d(12.246, 2.087, Rotation2d.fromDegrees(60))),
                        new AutoPoint(
                            new Pose2d(13.872, 1.903, Rotation2d.fromDegrees(135.88)),
                            Commands.runOnce(robotManager::stowRequest)),
                        new AutoPoint(
                            Points.LEFT_CORAL_STATION.redPose,
                            autoCommands.intakeStationWarmupCommand()))),
                autoCommands.intakeStationWithTimeoutCommand()),

        // SCORE L4 ON K
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_K)
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            SCORING_CONSTRAINTS,
                            new AutoPoint(
                                new Pose2d(14.506, 1.903, Rotation2d.fromDegrees(133.277))),
                            // REEF PIPE K
                            new AutoPoint(robotManager.autoAlign::getUsedScoringPose)),
                        false)
                    .until(robotManager.autoAlign::isTagAlignedDebounced)),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        Commands.waitSeconds(0.5)
            .andThen(robotManager::stowRequest)
            .alongWith(
                trailblazer.followSegment(
                    new AutoSegment(
                        INTAKING_CONSTRAINTS,
                        new AutoPoint(new Pose2d(14.284, 2.087, Rotation2d.fromDegrees(133.277))),
                        new AutoPoint(
                            new Pose2d(15.083, 1.439, Rotation2d.fromDegrees(133.277)),
                            Commands.runOnce(robotManager::stowRequest)),
                        new AutoPoint(
                            Points.LEFT_CORAL_STATION.redPose,
                            autoCommands.intakeStationWarmupCommand()))),
                autoCommands.intakeStationWithTimeoutCommand()),

        // SCORE L4 ON L
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_L)
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            SCORING_CONSTRAINTS,
                            new AutoPoint(
                                new Pose2d(14.954, 1.971, Rotation2d.fromDegrees(134.931))),
                            // REEF PIPE L
                            new AutoPoint(robotManager.autoAlign::getUsedScoringPose)),
                        false)
                    .until(robotManager.autoAlign::isTagAlignedDebounced)),
        autoCommands.l4ScoreAndReleaseCommand(),
        trailblazer.followSegment(
            new AutoSegment(
              SCORING_CONSTRAINTS,
                new AutoPoint(new Pose2d(13.998, 2.812, Rotation2d.fromDegrees(134.931))),
                new AutoPoint(
                  new Pose2d(14.284, 2.435, Rotation2d.fromDegrees(134.931)),
                    Commands.runOnce(robotManager::stowRequest)))));
  }
}
