package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.elevator.CoralStation;
import frc.robot.robot_manager.RobotManager;

public class AutoBlocks {
  /**
   * The offset used to calculate the position to go to before PIDing to the lineup pose. Ensures we
   * don't strafe into the scoring position, just a straight-on movement.
   */
  private static final Transform2d PIPE_LINEUP_OFFSET = new Transform2d(-0.4, 0, Rotation2d.kZero);

  private static final Transform2d PIPE_APPROACH_OFFSET = new Transform2d(-1, 0, Rotation2d.kZero);

  private static final Transform2d FRONT_STATION_APPROACH_OFFSET =
      new Transform2d(-0.8, 0, Rotation2d.kZero);

  private static final Transform2d BACK_STATION_APPROACH_OFFSET =
      new Transform2d(0.6, 0, Rotation2d.kZero);

  private static final AutoConstraintOptions BASE_CONSTRAINTS =
      new AutoConstraintOptions(4.5, 57, 4, 30);
  private static final AutoConstraintOptions SCORING_CONSTRAINTS =
      BASE_CONSTRAINTS.withMaxLinearVelocity(3).withMaxLinearAcceleration(3);

  private final Trailblazer trailblazer;
  private final RobotManager robotManager;
  private final AutoCommands autoCommands;

  public AutoBlocks(Trailblazer trailblazer, RobotManager robotManager, AutoCommands autoCommands) {
    this.trailblazer = trailblazer;
    this.robotManager = robotManager;
    this.autoCommands = autoCommands;
  }

  public Command scoreL4(ReefPipe pipe) {
    return Commands.sequence(
        trailblazer
            .followSegment(
                new AutoSegment(
                    BASE_CONSTRAINTS,
                    new AutoPoint(
                        () -> pipe.getPose(ReefPipeLevel.L4).transformBy(PIPE_APPROACH_OFFSET),
                        autoCommands.l4WarmupCommand(pipe)),
                    new AutoPoint(
                        () -> pipe.getPose(ReefPipeLevel.L4).transformBy(PIPE_LINEUP_OFFSET),
                        autoCommands.l4LineupCommand(pipe),
                        SCORING_CONSTRAINTS),
                    new AutoPoint(
                        () -> robotManager.autoAlign.getUsedScoringPose(pipe, ReefPipeLevel.L4))),
                false)
            .withDeadline(
                Commands.waitUntil(robotManager.autoAlign::isTagAlignedDebounced)
                    .andThen(autoCommands.l4ScoreAndReleaseCommand())),
        trailblazer.followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                new AutoPoint(
                    () -> robotManager.autoAlign.getUsedScoringPose(pipe, ReefPipeLevel.L4),
                    Commands.waitSeconds(0.25).andThen(robotManager::stowRequest)),
                new AutoPoint(
                    () -> pipe.getPose(ReefPipeLevel.L4).transformBy(PIPE_LINEUP_OFFSET)))));
  }

  public Command scorePreloadL4(Pose2d startingPose, ReefPipe pipe) {
    return Commands.sequence(
        Commands.runOnce(robotManager::rehomeRollRequest),
        trailblazer
            .followSegment(
                new AutoSegment(
                    BASE_CONSTRAINTS,
                    // Start on auto line
                    new AutoPoint(
                        startingPose,
                        autoCommands
                            .preloadCoralAfterRollHomed()
                            .andThen(autoCommands.l4WarmupCommand(pipe)),
                        BASE_CONSTRAINTS),
                    new AutoPoint(
                        () -> pipe.getPose(ReefPipeLevel.L4).transformBy(PIPE_LINEUP_OFFSET),
                        autoCommands.l4LineupCommand(pipe),
                        SCORING_CONSTRAINTS),
                    // Actually align to score
                    new AutoPoint(
                        () -> robotManager.autoAlign.getUsedScoringPose(pipe, ReefPipeLevel.L4),
                        autoCommands.l4LineupCommand(pipe),
                        SCORING_CONSTRAINTS)),
                false)
            .withDeadline(
                Commands.waitUntil(robotManager.autoAlign::isTagAlignedDebounced)
                    .andThen(autoCommands.l4ScoreAndReleaseCommand())),
        trailblazer.followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                // Start at the scoring position
                new AutoPoint(
                    () -> robotManager.autoAlign.getUsedScoringPose(pipe, ReefPipeLevel.L4),
                    Commands.waitSeconds(0.25).andThen(robotManager::stowRequest)),
                // Scoot back to the lineup position to finish the score
                new AutoPoint(
                    () -> pipe.getPose(ReefPipeLevel.L4).transformBy(PIPE_LINEUP_OFFSET)))));
  }

  public Command intakeStationFront(CoralStation station) {
    return trailblazer
        .followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                new AutoPoint(
                    station.frontLoadPose.transformBy(FRONT_STATION_APPROACH_OFFSET),
                    Commands.runOnce(robotManager::intakeStationFrontRequest)),
                new AutoPoint(station.frontLoadPose)),
            false)
        .withDeadline(autoCommands.waitForFrontIntakeDone());
  }

  public Command intakeStationBack(CoralStation station) {
    return trailblazer
        .followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                new AutoPoint(
                    station.backLoadPose.transformBy(BACK_STATION_APPROACH_OFFSET),
                    Commands.runOnce(robotManager::intakeStationBackRequest)),
                new AutoPoint(station.backLoadPose)),
            false)
        .withDeadline(autoCommands.waitForBackIntakeDone());
  }

  public Command intakeStationGround(Pose2d searchPose) {
    return trailblazer
        .followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                new AutoPoint(searchPose),
                new AutoPoint(
                    () -> robotManager.coralMap.getBestCoral().orElse(searchPose),
                    Commands.runOnce(robotManager::intakeAssistFloorCoralHorizontalRequest))),
            false)
        .withDeadline(autoCommands.waitForGroundIntakeDone());
  }
}
