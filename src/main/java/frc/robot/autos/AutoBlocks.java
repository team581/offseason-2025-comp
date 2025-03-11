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
  private static final Transform2d PIPE_APPROACH_OFFSET =
      new Transform2d(-0.4, 0, Rotation2d.kZero);

  private static final Transform2d STATION_APPROACH_OFFSET =
      new Transform2d(-0.6, 0, Rotation2d.kZero);

  private static final AutoConstraintOptions BASE_CONSTRAINTS =
      new AutoConstraintOptions(3.5, 57, 4, 30);
  private static final AutoConstraintOptions SCORING_CONSTRAINTS =
      BASE_CONSTRAINTS.withMaxLinearVelocity(1.5);

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
                    SCORING_CONSTRAINTS,
                    new AutoPoint(
                        () -> pipe.getPose(ReefPipeLevel.L4).transformBy(PIPE_APPROACH_OFFSET),
autoCommands.l4WarmupCommand(pipe)),
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
                    () -> pipe.getPose(ReefPipeLevel.L4).transformBy(PIPE_APPROACH_OFFSET)))));
  }

  public Command scorePreloadL4(Pose2d startingPose, ReefPipe pipe) {
    return Commands.sequence(
        Commands.runOnce(robotManager::rehomeRollRequest),
        trailblazer
            .followSegment(
                new AutoSegment(
                    SCORING_CONSTRAINTS,
                    new AutoPoint(startingPose),
                    new AutoPoint(
                        () -> pipe.getPose(ReefPipeLevel.L4).transformBy(PIPE_APPROACH_OFFSET),
                        autoCommands
                            .preloadCoralAfterRollHomed()
                            .andThen(autoCommands.l4WarmupCommand(pipe))),
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
                    () -> pipe.getPose(ReefPipeLevel.L4).transformBy(PIPE_APPROACH_OFFSET)))));
  }

  public Command intakeStationFront(CoralStation station) {
    return trailblazer
        .followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                new AutoPoint(
                    station.frontLoadPose.transformBy(STATION_APPROACH_OFFSET),
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
                    station.backLoadPose.transformBy(STATION_APPROACH_OFFSET),
                    Commands.runOnce(robotManager::intakeStationBackRequest)),
                new AutoPoint(station.backLoadPose)),
            false)
        .until(autoCommands::isSmartStowing);
  }
}
