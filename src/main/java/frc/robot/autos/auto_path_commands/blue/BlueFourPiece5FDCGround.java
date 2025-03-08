package frc.robot.autos.auto_path_commands.blue;

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

public class BlueFourPiece5FDCGround extends BaseAuto {
  private static final AutoConstraintOptions INTAKING_CONSTRAINTS =
      new AutoConstraintOptions(4.75, 57, 4, 30);
  private static final AutoConstraintOptions SCORING_CONSTRAINTS =
      new AutoConstraintOptions(2, 57, 4, 30);

  public BlueFourPiece5FDCGround(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_5_AND_2.bluePose;
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
                    new AutoPoint(Points.START_5_AND_2.bluePose, INTAKING_CONSTRAINTS),
                    new AutoPoint(
                        new Pose2d(5.765, 2, Rotation2d.fromDegrees(120)),
                        autoCommands
                            .preloadCoralAfterRollHomed()
                            .andThen(autoCommands.l4WarmupCommand(ReefPipe.PIPE_F)),
                        new AutoConstraintOptions(1.5, 57, 4, 30)),
                    new AutoPoint(
                        robotManager.autoAlign::getUsedScoringPose,
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
                        new Pose2d(5.418, 2.243, Rotation2d.fromDegrees(45)),
                        Commands.waitSeconds(0.25).andThen(robotManager::stowRequest)),
                    new AutoPoint(
                        () ->
                            robotManager
                                .coralMap
                                .getBestCoral()
                                .orElse(new Pose2d(2.906, 2.031, Rotation2d.fromDegrees(-145.0))),
                        Commands.runOnce(
                            () -> robotManager.intakeAssistFloorCoralHorizontalRequest()),
                        INTAKING_CONSTRAINTS)),
                false)
            .raceWith(waitForGroundIntakeDone()),

        // SCORE L4 ON D
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_D)
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            SCORING_CONSTRAINTS,
                            new AutoPoint(
                                new Pose2d(2.636, 1.553, Rotation2d.fromDegrees(47)),
                                new AutoConstraintOptions(2.3, 57, 4, 30)),
                            new AutoPoint(
                                new Pose2d(3.266, 2.087, Rotation2d.fromDegrees(47)),
                                new AutoConstraintOptions(1.5, 57, 4, 30)),
                            // REEF PIPE K
                            new AutoPoint(
                                robotManager.autoAlign::getUsedScoringPose,
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
                        new Pose2d(3.266, 2.087, Rotation2d.fromDegrees(47)),
                        Commands.waitSeconds(0.25).andThen(robotManager::stowRequest)),
                    new AutoPoint(
                        () ->
                            robotManager
                                .coralMap
                                .getBestCoral()
                                .orElse(new Pose2d(2.906, 2.031, Rotation2d.fromDegrees(-145.0))),
                        Commands.runOnce(
                            () -> robotManager.intakeAssistFloorCoralHorizontalRequest()),
                        INTAKING_CONSTRAINTS)),
                false)
            .raceWith(waitForGroundIntakeDone()),

        // SCORE L4 ON C
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_C)
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            SCORING_CONSTRAINTS,
                            new AutoPoint(
                                new Pose2d(2.636, 1.801, Rotation2d.fromDegrees(47)),
                                new AutoConstraintOptions(2.3, 57, 4, 30)),
                            new AutoPoint(
                                new Pose2d(3.266, 2.435, Rotation2d.fromDegrees(46)),
                                new AutoConstraintOptions(1.5, 57, 4, 30)),
                            // REEF PIPE L
                            new AutoPoint(
                                robotManager.autoAlign::getUsedScoringPose,
                                new AutoConstraintOptions(1.5, 57, 4, 30))),
                        false)
                    .until(autoCommands::alignedForScore)),
        autoCommands.l4ScoreAndReleaseCommand(),

        // DRIVE BACK & STOW
        trailblazer.followSegment(
            new AutoSegment(
                INTAKING_CONSTRAINTS,
                new AutoPoint(new Pose2d(3.552, 2.812, Rotation2d.fromDegrees(46))),
                new AutoPoint(
                    new Pose2d(3.266, 2.435, Rotation2d.fromDegrees(46)),
                    autoCommands.stowRequest()))));
  }
}
