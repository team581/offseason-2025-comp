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

public class RedFourPiece2IKLGroundAuto extends BaseAuto {
  private static final AutoConstraintOptions INTAKING_CONSTRAINTS =
      new AutoConstraintOptions(3.5, 57, 3.5, 30);
  private static final AutoConstraintOptions SCORING_CONSTRAINTS =
      new AutoConstraintOptions(2, 57, 4, 30);

  public RedFourPiece2IKLGroundAuto(RobotManager robotManager, Trailblazer trailblazer) {
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

        // INTAKE GROUND WITH CORAL MAP
        trailblazer
            .followSegment(
                new AutoSegment(
                    INTAKING_CONSTRAINTS,
                    new AutoPoint(
                        new Pose2d(12.132, 2.243, Rotation2d.fromDegrees(60)),
                        Commands.waitSeconds(0.25).andThen(robotManager::stowRequest)),
                    new AutoPoint(new Pose2d(12.888, 1.871, Rotation2d.fromDegrees(-31.0))),
                    new AutoPoint(
                        () ->
                            robotManager
                                .coralMap
                                .getBestCoral()
                                .orElse(new Pose2d(15.151, 1.244, Rotation2d.fromDegrees(-20.0))),
                        Commands.runOnce(
                            () -> robotManager.intakeAssistFloorCoralHorizontalRequest()),
                        INTAKING_CONSTRAINTS)),
                false)
            .withDeadline(waitForGroundIntakeDone()),

        // SCORE L4 ON K

        trailblazer
            .followSegment(
                new AutoSegment(
                    new AutoPoint(
                        new Pose2d(14.038, 1.888, Rotation2d.fromDegrees(120.0)),
                        new AutoConstraintOptions(3.5, 57, 2.0, 30)),
                    // REEF PIPE K
                    new AutoPoint(
                        () ->
                            robotManager.autoAlign.getUsedScoringPose(
                                ReefPipe.PIPE_K, ReefPipeLevel.L4),
                        autoCommands.l4WarmupCommand(ReefPipe.PIPE_K),
                        new AutoConstraintOptions(1.5, 57, 2.0, 30))),
                false)
            .until(autoCommands::alignedForScore),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE GROUND CORAL MAP
        trailblazer
            .followSegment(
                new AutoSegment(
                    INTAKING_CONSTRAINTS,
                    new AutoPoint(
                        new Pose2d(14.284, 2.087, Rotation2d.fromDegrees(100)),
                        Commands.waitSeconds(0.25).andThen(robotManager::stowRequest)),
                    new AutoPoint(new Pose2d(14.25, 2.0, Rotation2d.fromDegrees(-35))),
                    new AutoPoint(
                        () ->
                            robotManager
                                .coralMap
                                .getBestCoral()
                                .orElse(new Pose2d(15.151, 1.244, Rotation2d.fromDegrees(-20.0))),
                        Commands.runOnce(
                            () -> robotManager.intakeAssistFloorCoralHorizontalRequest()),
                        INTAKING_CONSTRAINTS)),
                false)
            .withDeadline(waitForGroundIntakeDone()),

        // SCORE L4 ON L
        trailblazer
            .followSegment(
                new AutoSegment(
                    new AutoPoint(
                        new Pose2d(14.517, 2.047, Rotation2d.fromDegrees(120.0)),
                        new AutoConstraintOptions(3.5, 57, 2.0, 30)),
                    // REEF PIPE L
                    new AutoPoint(
                        () ->
                            robotManager.autoAlign.getUsedScoringPose(
                                ReefPipe.PIPE_L, ReefPipeLevel.L4),
                        autoCommands.l4WarmupCommand(ReefPipe.PIPE_L),
                        new AutoConstraintOptions(1.5, 57, 2.0, 30))),
                false)
            .until(autoCommands::alignedForScore),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE GROUND CORAL MAP
        trailblazer
            .followSegment(
                new AutoSegment(
                    INTAKING_CONSTRAINTS,
                    new AutoPoint(
                        new Pose2d(14.183, 2.385, Rotation2d.fromDegrees(100)),
                        Commands.waitSeconds(0.25).andThen(robotManager::stowRequest)),
                    new AutoPoint(new Pose2d(14.188, 1.974, Rotation2d.fromDegrees(-40))),
                    new AutoPoint(
                        () ->
                            robotManager
                                .coralMap
                                .getBestCoral()
                                .orElse(new Pose2d(15.151, 1.244, Rotation2d.fromDegrees(-20.0))),
                        Commands.runOnce(
                            () -> robotManager.intakeAssistFloorCoralHorizontalRequest()),
                        INTAKING_CONSTRAINTS)),
                false)
            .withDeadline(waitForGroundIntakeDone()),

        // SCORE L4 ON A
        trailblazer
            .followSegment(
                new AutoSegment(
                    new AutoPoint(
                        new Pose2d(15.288, 3.858, Rotation2d.fromDegrees(180.0)),
                        new AutoConstraintOptions(3.5, 57, 2.0, 30)),
                    // REEF PIPE A
                    new AutoPoint(
                        () ->
                            robotManager.autoAlign.getUsedScoringPose(
                                ReefPipe.PIPE_A, ReefPipeLevel.L4),
                        autoCommands.l4WarmupCommand(ReefPipe.PIPE_A),
                        new AutoConstraintOptions(1.5, 57, 2.0, 30))),
                false)
            .until(autoCommands::alignedForScore),
        autoCommands.l4ScoreAndReleaseCommand(),

        // DRIVE BACK & STOW
        trailblazer.followSegment(
            new AutoSegment(
                INTAKING_CONSTRAINTS,
                new AutoPoint(new Pose2d(14.834, 3.788, Rotation2d.fromDegrees(180.0))),
                new AutoPoint(
                    new Pose2d(15.354, 3.749, Rotation2d.fromDegrees(180.0)),
                    autoCommands.stowRequest()))));
  }
}
