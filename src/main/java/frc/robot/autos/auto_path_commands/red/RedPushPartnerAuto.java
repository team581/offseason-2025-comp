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

public class RedPushPartnerAuto extends BaseAuto {
  private static final AutoConstraintOptions INTAKING_CONSTRAINTS =
      new AutoConstraintOptions(4.75, 57, 4, 30);
  private static final AutoConstraintOptions SCORING_CONSTRAINTS =
      new AutoConstraintOptions(2.3, 57, 4, 30);

  public RedPushPartnerAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_2_AND_5.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        Commands.runOnce(robotManager::rehomeRollRequest),
        // SCORE L4 ON I
        trailblazer.followSegment(
            // PUSH PARTNER
            new AutoSegment(
                INTAKING_CONSTRAINTS,
                new AutoPoint(Points.START_2_AND_5.redPose),
                new AutoPoint(new Pose2d(9.963, 1.903, Rotation2d.fromDegrees(0.0))))),
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
                            .andThen(autoCommands.l4WarmupCommand(ReefPipe.PIPE_I)),
                        new AutoConstraintOptions(1.5, 57, 4, 30)),
                    new AutoPoint(
                        () ->
                            robotManager.autoAlign.getUsedScoringPose(
                                ReefPipe.PIPE_I, ReefPipeLevel.L4),
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
                        new Pose2d(12.132, 2.243, Rotation2d.fromDegrees(135.88)),
                        Commands.waitSeconds(0.25).andThen(robotManager::stowRequest)),
                    //        new AutoConstraintOptions(4, 57, 4, 30)),
                    new AutoPoint(new Pose2d(13.636, 1.439, Rotation2d.fromDegrees(135.88))),
                    new AutoPoint(
                        new Pose2d(15.241, 1.107, Rotation2d.fromDegrees(135.88)),
                        autoCommands.intakeStationWarmupCommand(),
                        new AutoConstraintOptions(3, 57, 4, 30)),
                    new AutoPoint(Points.LEFT_CORAL_STATION.redPose)),
                false)
            .until(autoCommands::isSmartStowing),

        // SCORE L4 ON K
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_K)
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            SCORING_CONSTRAINTS,
                            new AutoPoint(
                                new Pose2d(14.914, 1.553, Rotation2d.fromDegrees(133.277)),
                                new AutoConstraintOptions(2.3, 57, 4, 30)),
                            new AutoPoint(
                                new Pose2d(14.284, 2.087, Rotation2d.fromDegrees(133.277)),
                                new AutoConstraintOptions(1.5, 57, 4, 30)),
                            // REEF PIPE K
                            new AutoPoint(
                                () ->
                                    robotManager.autoAlign.getUsedScoringPose(
                                        ReefPipe.PIPE_K, ReefPipeLevel.L4),
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
                        new Pose2d(14.284, 2.087, Rotation2d.fromDegrees(133.277)),
                        Commands.waitSeconds(0.25).andThen(robotManager::stowRequest)),
                    new AutoPoint(
                        new Pose2d(15.083, 1.439, Rotation2d.fromDegrees(133.277)),
                        autoCommands.intakeStationWarmupCommand(),
                        new AutoConstraintOptions(3, 57, 4, 30)),
                    new AutoPoint(Points.LEFT_CORAL_STATION.redPose)),
                false)
            .until(autoCommands::isSmartStowing),

        // SCORE L4 ON L
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_L)
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            SCORING_CONSTRAINTS,
                            new AutoPoint(
                                new Pose2d(14.914, 1.801, Rotation2d.fromDegrees(133.277)),
                                new AutoConstraintOptions(2.3, 57, 4, 30)),
                            new AutoPoint(
                                new Pose2d(14.284, 2.435, Rotation2d.fromDegrees(134.931)),
                                new AutoConstraintOptions(1.5, 57, 4, 30)),
                            // REEF PIPE L
                            new AutoPoint(
                                () ->
                                    robotManager.autoAlign.getUsedScoringPose(
                                        ReefPipe.PIPE_L, ReefPipeLevel.L4),
                                new AutoConstraintOptions(1.5, 57, 4, 30))),
                        false)
                    .until(autoCommands::alignedForScore)),
        autoCommands.l4ScoreAndReleaseCommand(),

        // DRIVE BACK & STOW
        trailblazer.followSegment(
            new AutoSegment(
                INTAKING_CONSTRAINTS,
                new AutoPoint(new Pose2d(13.998, 2.812, Rotation2d.fromDegrees(134.931))),
                new AutoPoint(
                    new Pose2d(14.284, 2.435, Rotation2d.fromDegrees(134.931)),
                    autoCommands.stowRequest()))));
  }
}
