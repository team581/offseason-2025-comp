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
      new AutoConstraintOptions(4, 57, 4, 30);
  private static final AutoConstraintOptions SCORING_CONSTRAINTS =
      new AutoConstraintOptions(2, 57, 4, 30);

  public FourPiece2IJKLAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getBlueStartingPose() {
    return Points.START_2_AND_5.bluePose;
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
                    new AutoPoint(Points.START_2_AND_5.bluePose, INTAKING_CONSTRAINTS),
                    new AutoPoint(
                            new Pose2d(11.785, 2.0, Rotation2d.fromDegrees(60)),
                            autoCommands
                                .preloadCoralAfterRollHomed()
                                .andThen(autoCommands.l4WarmupCommand(ReefPipe.PIPE_I)))
                        .pathflipped(),
                    new AutoPoint(robotManager.autoAlign::getUsedScoringPose)),
                false)
            .until(
                () ->
                    robotManager.autoAlign.isTagAlignedDebounced()
                        && robotManager.imu.isFlatDebounced()),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer
            .followSegment(
                new AutoSegment(
                    new AutoPoint(
                            new Pose2d(12.132, 2.243, Rotation2d.fromDegrees(60)),
                            Commands.waitSeconds(0.25).andThen(robotManager::stowRequest),
                            new AutoConstraintOptions(4, 57, 4, 30))
                        .pathflipped(),
                    new AutoPoint(
                            new Pose2d(13.636, 1.439, Rotation2d.fromDegrees(135.88)),
                            new AutoConstraintOptions(2, 57, 4, 30))
                        .pathflipped(),
                    new AutoPoint(
                        Points.LEFT_CORAL_STATION.bluePose,
                        autoCommands.intakeStationWarmupCommand(),
                        new AutoConstraintOptions(1, 57, 4, 30))),
                false)
            .until(
                () ->
                    robotManager.getState() == RobotState.SMART_STOW_1
                        || robotManager.getState() == RobotState.SMART_STOW_2),

        // SCORE L4 ON J
        trailblazer
            .followSegment(
                new AutoSegment(
                    SCORING_CONSTRAINTS,
                    new AutoPoint(
                            new Pose2d(14.119, 1.107, Rotation2d.fromDegrees(90.5)),
                            autoCommands.l4WarmupCommand(ReefPipe.PIPE_J),
                            INTAKING_CONSTRAINTS)
                        .pathflipped(),
                    new AutoPoint(new Pose2d(12.246, 1.553, Rotation2d.fromDegrees(60.0)))
                        .pathflipped(),
                    new AutoPoint(robotManager.autoAlign::getUsedScoringPose)),
                false)
            .until(
                () ->
                    robotManager.autoAlign.isTagAlignedDebounced()
                        && robotManager.imu.isFlatDebounced()),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        // TODO: Trailblazer path shouldn't end at position, it should keep PIDing while waiting for
        // a coral
        trailblazer
            .followSegment(
                new AutoSegment(
                    INTAKING_CONSTRAINTS,
                    new AutoPoint(
                            new Pose2d(12.246, 2.087, Rotation2d.fromDegrees(60)),
                            Commands.waitSeconds(0.25).andThen(robotManager::stowRequest))
                        .pathflipped(),
                    new AutoPoint(new Pose2d(13.872, 1.903, Rotation2d.fromDegrees(135.88)))
                        .pathflipped(),
                    new AutoPoint(
                        Points.LEFT_CORAL_STATION.bluePose,
                        autoCommands.intakeStationWarmupCommand())),
                false)
            .until(
                () ->
                    robotManager.getState() == RobotState.SMART_STOW_1
                        || robotManager.getState() == RobotState.SMART_STOW_2),

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
                    .until(
                        () ->
                            robotManager.autoAlign.isTagAlignedDebounced()
                                && robotManager.imu.isFlatDebounced())),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer
            .followSegment(
                new AutoSegment(
                    INTAKING_CONSTRAINTS,
                    new AutoPoint(
                            new Pose2d(14.284, 2.087, Rotation2d.fromDegrees(133.277)),
                            Commands.waitSeconds(0.25).andThen(robotManager::stowRequest))
                        .pathflipped(),
                    new AutoPoint(new Pose2d(15.083, 1.439, Rotation2d.fromDegrees(133.277)))
                        .pathflipped(),
                    new AutoPoint(
                        Points.LEFT_CORAL_STATION.bluePose,
                        autoCommands.intakeStationWarmupCommand())),
                false)
            .until(
                () ->
                    robotManager.getState() == RobotState.SMART_STOW_1
                        || robotManager.getState() == RobotState.SMART_STOW_2),

        // SCORE L4 ON L
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_L)
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
                    .until(
                        () ->
                            robotManager.autoAlign.isTagAlignedDebounced()
                                && robotManager.imu.isFlatDebounced())),
        autoCommands.l4ScoreAndReleaseCommand(),

        // DRIVE BACK & STOW
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
    return Points.START_2_AND_5.redPose;
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
                    new AutoPoint(Points.START_2_AND_5.redPose, INTAKING_CONSTRAINTS),
                    new AutoPoint(
                        new Pose2d(11.785, 2.0, Rotation2d.fromDegrees(60)),
                        autoCommands
                            .preloadCoralAfterRollHomed()
                            .andThen(autoCommands.l4WarmupCommand(ReefPipe.PIPE_I))),
                    new AutoPoint(robotManager.autoAlign::getUsedScoringPose)),
                false)
            .until(
                () ->
                    robotManager.autoAlign.isTagAlignedDebounced()
                        && robotManager.imu.isFlatDebounced()),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer
            .followSegment(
                new AutoSegment(
                    new AutoPoint(
                        new Pose2d(12.132, 2.243, Rotation2d.fromDegrees(60)),
                        Commands.waitSeconds(0.25).andThen(robotManager::stowRequest)),
                //        new AutoConstraintOptions(4, 57, 4, 30)),
                    new AutoPoint(
                        new Pose2d(13.636, 1.439, Rotation2d.fromDegrees(135.88))),
                //        new AutoConstraintOptions(2, 57, 4, 30)),
                    new AutoPoint(
                        Points.LEFT_CORAL_STATION.redPose,
                        autoCommands.intakeStationWarmupCommand())),
                false)
            .until(
                () ->
                    robotManager.getState() == RobotState.SMART_STOW_1
                        || robotManager.getState() == RobotState.SMART_STOW_2),

        // SCORE L4 ON J
        trailblazer
            .followSegment(
                new AutoSegment(
                    SCORING_CONSTRAINTS,
                    new AutoPoint(
                        new Pose2d(14.103, 1.0, Rotation2d.fromDegrees(90.5)),
                        autoCommands.l4WarmupCommand(ReefPipe.PIPE_J),
                        INTAKING_CONSTRAINTS),
                    new AutoPoint(new Pose2d(12.246, 1.244, Rotation2d.fromDegrees(60.0))),
                    new AutoPoint(robotManager.autoAlign::getUsedScoringPose)),
                false)
            .until(
                () ->
                    robotManager.autoAlign.isTagAlignedDebounced()
                        && robotManager.imu.isFlatDebounced()),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer
            .followSegment(
                new AutoSegment(
                    INTAKING_CONSTRAINTS,
                    new AutoPoint(
                        new Pose2d(12.246, 2.087, Rotation2d.fromDegrees(60)),
                        Commands.waitSeconds(0.25).andThen(robotManager::stowRequest)),
                    new AutoPoint(new Pose2d(13.872, 1.903, Rotation2d.fromDegrees(135.88))),
                    new AutoPoint(
                        Points.LEFT_CORAL_STATION.redPose,
                        autoCommands.intakeStationWarmupCommand())),
                false)
            .until(
                () ->
                    robotManager.getState() == RobotState.SMART_STOW_1
                        || robotManager.getState() == RobotState.SMART_STOW_2),

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
                    .until(
                        () ->
                            robotManager.autoAlign.isTagAlignedDebounced()
                                && robotManager.imu.isFlatDebounced())),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer
            .followSegment(
                new AutoSegment(
                    INTAKING_CONSTRAINTS,
                    new AutoPoint(
                        new Pose2d(14.284, 2.087, Rotation2d.fromDegrees(133.277)),
                        Commands.waitSeconds(0.25).andThen(robotManager::stowRequest)),
                    new AutoPoint(new Pose2d(15.083, 1.439, Rotation2d.fromDegrees(133.277))),
                    new AutoPoint(
                        Points.LEFT_CORAL_STATION.redPose,
                        autoCommands.intakeStationWarmupCommand())),
                false)
            .until(
                () ->
                    robotManager.getState() == RobotState.SMART_STOW_1
                        || robotManager.getState() == RobotState.SMART_STOW_2),

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
                    .until(
                        () ->
                            robotManager.autoAlign.isTagAlignedDebounced()
                                && robotManager.imu.isFlatDebounced())),
        autoCommands.l4ScoreAndReleaseCommand(),

        // DRIVE BACK & STOW
        trailblazer.followSegment(
            new AutoSegment(
                SCORING_CONSTRAINTS,
                new AutoPoint(new Pose2d(13.998, 2.812, Rotation2d.fromDegrees(134.931))),
                new AutoPoint(
                    new Pose2d(14.284, 2.435, Rotation2d.fromDegrees(134.931)),
                    Commands.runOnce(robotManager::stowRequest)))));
  }
}
