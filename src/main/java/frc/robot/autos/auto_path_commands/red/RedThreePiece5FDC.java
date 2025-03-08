package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.robot_manager.RobotManager;

public class RedThreePiece5FDC extends BaseAuto {
  private static final AutoConstraintOptions INTAKING_CONSTRAINTS =
      new AutoConstraintOptions(4.75, 57, 4, 30);
  private static final AutoConstraintOptions SCORING_CONSTRAINTS =
      new AutoConstraintOptions(2, 57, 4, 30);

  public RedThreePiece5FDC(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_5_AND_2.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        Commands.runOnce(robotManager::rehomeRollRequest),
        // SCORE L4 ON F
        trailblazer
            .followSegment(
                new AutoSegment(
                    SCORING_CONSTRAINTS,
                    new AutoPoint(Points.START_5_AND_2.redPose, INTAKING_CONSTRAINTS),
                    new AutoPoint(
                        new Pose2d(11.785, 6.05, Rotation2d.fromDegrees(-60)),
                        autoCommands
                            .preloadCoralAfterRollHomed()
                            .andThen(autoCommands.l4WarmupCommand(ReefPipe.PIPE_F)),
                        new AutoConstraintOptions(1.5, 57, 4, 30)),
                    new AutoPoint(
                        () ->
                            robotManager.autoAlign.getUsedScoringPose(
                                ReefPipe.PIPE_F, ReefPipeLevel.L4),
                        new AutoConstraintOptions(1.5, 57, 4, 30))),
                false)
            .until(autoCommands::alignedForScore),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer
            .followSegment(
                new AutoSegment(
                    INTAKING_CONSTRAINTS,
                    new AutoPoint(
                        new Pose2d(12.132, 5.807, Rotation2d.fromDegrees(-135)),
                        Commands.waitSeconds(0.25).andThen(robotManager::stowRequest)),
                    //        new AutoConstraintOptions(4, 57, 4, 30)),
                    new AutoPoint(new Pose2d(13.636, 6.611, Rotation2d.fromDegrees(-135))),
                    new AutoPoint(
                        new Pose2d(15.241, 6.943, Rotation2d.fromDegrees(-135)),
                        autoCommands.intakeStationWarmupCommand(),
                        new AutoConstraintOptions(3, 57, 4, 30)),
                    new AutoPoint(Points.RIGHT_CORAL_STATION.redPose)),
                false)
            .until(autoCommands::isSmartStowing),

        // SCORE L4 ON D
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_D)
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            SCORING_CONSTRAINTS,
                            new AutoPoint(
                                new Pose2d(14.914, 6.497, Rotation2d.fromDegrees(-133)),
                                new AutoConstraintOptions(2.3, 57, 4, 30)),
                            new AutoPoint(
                                new Pose2d(14.284, 5.963, Rotation2d.fromDegrees(-133)),
                                new AutoConstraintOptions(1.5, 57, 4, 30)),
                            // REEF PIPE D
                            new AutoPoint(
                                () ->
                                    robotManager.autoAlign.getUsedScoringPose(
                                        ReefPipe.PIPE_D, ReefPipeLevel.L4),
                                new AutoConstraintOptions(1.5, 57, 4, 30))),
                        false)
                    .until(autoCommands::alignedForScore)),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer
            .followSegment(
                new AutoSegment(
                    INTAKING_CONSTRAINTS,
                    new AutoPoint(
                        new Pose2d(14.284, 5.963, Rotation2d.fromDegrees(-133)),
                        Commands.waitSeconds(0.25).andThen(robotManager::stowRequest)),
                    new AutoPoint(
                        new Pose2d(15.083, 6.611, Rotation2d.fromDegrees(-133)),
                        autoCommands.intakeStationWarmupCommand(),
                        new AutoConstraintOptions(3, 57, 4, 30)),
                    new AutoPoint(Points.RIGHT_CORAL_STATION.redPose)),
                false)
            .until(autoCommands::isSmartStowing),

        // SCORE L4 ON C
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_C)
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            SCORING_CONSTRAINTS,
                            new AutoPoint(
                                new Pose2d(14.914, 6.249, Rotation2d.fromDegrees(-133)),
                                new AutoConstraintOptions(2.3, 57, 4, 30)),
                            new AutoPoint(
                                new Pose2d(14.284, 5.615, Rotation2d.fromDegrees(-134)),
                                new AutoConstraintOptions(1.5, 57, 4, 30)),
                            // REEF PIPE C
                            new AutoPoint(
                                () ->
                                    robotManager.autoAlign.getUsedScoringPose(
                                        ReefPipe.PIPE_C, ReefPipeLevel.L4),
                                new AutoConstraintOptions(1.5, 57, 4, 30))),
                        false)
                    .until(autoCommands::alignedForScore)),
        autoCommands.l4ScoreAndReleaseCommand(),

        // DRIVE BACK & STOW
        trailblazer.followSegment(
            new AutoSegment(
                INTAKING_CONSTRAINTS,
                new AutoPoint(new Pose2d(13.998, 5.238, Rotation2d.fromDegrees(-134))),
                new AutoPoint(
                    new Pose2d(14.284, 5.615, Rotation2d.fromDegrees(-134)),
                    autoCommands.stowRequest()))));
  }
}
