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

public class RedFourPiece5FEDC extends BaseAuto {
  private static final AutoConstraintOptions INTAKING_CONSTRAINTS =
      new AutoConstraintOptions(4, 57, 4, 30);
  private static final AutoConstraintOptions SCORING_CONSTRAINTS =
      new AutoConstraintOptions(2, 57, 4, 30);

  public RedFourPiece5FEDC(RobotManager robotManager, Trailblazer trailblazer) {
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
        // SCORE L4 ON I
        trailblazer
            .followSegment(
                new AutoSegment(
                    SCORING_CONSTRAINTS,
                    new AutoPoint(Points.START_5_AND_2.redPose, INTAKING_CONSTRAINTS),
                    new AutoPoint(
                        new Pose2d(11.785, 5.428, Rotation2d.fromDegrees(-60)),
                        autoCommands
                            .preloadCoralAfterRollHomed()
                            .andThen(autoCommands.l4WarmupCommand(ReefPipe.PIPE_F))),
                    new AutoPoint(
                        () ->
                            robotManager.autoAlign.getUsedScoringPose(
                                ReefPipe.PIPE_I, ReefPipeLevel.L4))),
                false)
            .until(autoCommands::alignedForScore),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer
            .followSegment(
                new AutoSegment(
                    new AutoPoint(
                        new Pose2d(12.132, 5.807, Rotation2d.fromDegrees(-60)),
                        Commands.waitSeconds(0.25).andThen(robotManager::stowRequest),
                        new AutoConstraintOptions(4, 57, 4, 30)),
                    new AutoPoint(
                        new Pose2d(13.842, 6.393, Rotation2d.fromDegrees(-135.878)),
                        new AutoConstraintOptions(2, 57, 4, 30)),
                    new AutoPoint(
                        Points.RIGHT_CORAL_STATION.redPose,
                        autoCommands.intakeStationWarmupCommand(),
                        new AutoConstraintOptions(1, 57, 4, 30))),
                false)
            .until(autoCommands::isSmartStowing),

        // SCORE L4 ON J
        trailblazer
            .followSegment(
                new AutoSegment(
                    SCORING_CONSTRAINTS,
                    new AutoPoint(
                        new Pose2d(14.119, 6.943, Rotation2d.fromDegrees(-90.5)),
                        autoCommands.l4WarmupCommand(ReefPipe.PIPE_E),
                        INTAKING_CONSTRAINTS),
                    new AutoPoint(new Pose2d(12.246, 6.4974, Rotation2d.fromDegrees(-60.0))),
                    new AutoPoint(
                        () ->
                            robotManager.autoAlign.getUsedScoringPose(
                                ReefPipe.PIPE_J, ReefPipeLevel.L4))),
                false)
            .until(autoCommands::alignedForScore),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer
            .followSegment(
                new AutoSegment(
                    INTAKING_CONSTRAINTS,
                    new AutoPoint(
                        new Pose2d(12.246, 5.807, Rotation2d.fromDegrees(-60)),
                        Commands.waitSeconds(0.25).andThen(robotManager::stowRequest)),
                    new AutoPoint(new Pose2d(13.998, 6.393, Rotation2d.fromDegrees(-135.88))),
                    new AutoPoint(
                        Points.RIGHT_CORAL_STATION.redPose,
                        autoCommands.intakeStationWarmupCommand())),
                false)
            .until(autoCommands::isSmartStowing),

        // SCORE L4 ON K
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_D)
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            SCORING_CONSTRAINTS,
                            new AutoPoint(
                                new Pose2d(14.506, 6.147, Rotation2d.fromDegrees(-133.277))),
                            // REEF PIPE K
                            new AutoPoint(
                                () ->
                                    robotManager.autoAlign.getUsedScoringPose(
                                        ReefPipe.PIPE_K, ReefPipeLevel.L4))),
                        false)
                    .until(autoCommands::alignedForScore)),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer
            .followSegment(
                new AutoSegment(
                    INTAKING_CONSTRAINTS,
                    new AutoPoint(
                        new Pose2d(14.284, 5.963, Rotation2d.fromDegrees(-133.277)),
                        Commands.waitSeconds(0.25).andThen(robotManager::stowRequest)),
                    new AutoPoint(new Pose2d(15.083, 6.611, Rotation2d.fromDegrees(-133.277))),
                    new AutoPoint(
                        Points.RIGHT_CORAL_STATION.redPose,
                        autoCommands.intakeStationWarmupCommand())),
                false)
            .until(autoCommands::isSmartStowing),

        // SCORE L4 ON L
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_C)
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            SCORING_CONSTRAINTS,
                            new AutoPoint(
                                new Pose2d(14.954, 6.079, Rotation2d.fromDegrees(-134.931))),
                            // REEF PIPE L
                            new AutoPoint(
                                () ->
                                    robotManager.autoAlign.getUsedScoringPose(
                                        ReefPipe.PIPE_L, ReefPipeLevel.L4))),
                        false)
                    .until(autoCommands::alignedForScore)),
        autoCommands.l4ScoreAndReleaseCommand(),

        // DRIVE BACK & STOW
        trailblazer.followSegment(
            new AutoSegment(
                SCORING_CONSTRAINTS,
                new AutoPoint(new Pose2d(13.998, 5.328, Rotation2d.fromDegrees(-134.931))),
                new AutoPoint(
                    new Pose2d(14.284, 5.615, Rotation2d.fromDegrees(-134.931)),
                    autoCommands.stowRequest()))));
  }
}
