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

public class BlueFrontThreePiece2IKLAuto extends BaseAuto {
  private static final AutoConstraintOptions INTAKING_CONSTRAINTS =
      new AutoConstraintOptions(4.75, 57, 4, 30);
  private static final AutoConstraintOptions SCORING_CONSTRAINTS =
      new AutoConstraintOptions(2, 57, 4, 30);

  public BlueFrontThreePiece2IKLAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_2_AND_5.bluePose;
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
                    new AutoPoint(Points.START_2_AND_5.bluePose, INTAKING_CONSTRAINTS),
                    new AutoPoint(
                        new Pose2d(5.765, 6.05, Rotation2d.fromDegrees(-120)),
                        autoCommands
                            .preloadCoralAfterRollHomed()
                            .andThen(autoCommands.l4WarmupCommand(ReefPipe.PIPE_I)),
                        new AutoConstraintOptions(1.5, 57, 4, 30)),
                    new AutoPoint(
                        robotManager.autoAlign::getUsedScoringPose,
                        new AutoConstraintOptions(1.5, 57, 4, 30))),
                false)
            .until(autoCommands::alignedForScore),
        autoCommands.l4ScoreAndReleaseCommand(),
        autoCommands
            .waitThenStow()
            .alongWith(
                // INTAKE STATION
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            INTAKING_CONSTRAINTS,
                            new AutoPoint(new Pose2d(5.418, 5.807, Rotation2d.fromDegrees(-44.12))),
                            //        new AutoConstraintOptions(4, 57, 4, 30)),
                            new AutoPoint(new Pose2d(3.914, 6.611, Rotation2d.fromDegrees(134.12))),
                            new AutoPoint(
                                new Pose2d(2.309, 6.943, Rotation2d.fromDegrees(134.12)),
                                autoCommands.intakeStationWarmupCommand(),
                                new AutoConstraintOptions(3, 57, 4, 30)),
                            new AutoPoint(Points.LEFT_CORAL_STATION.bluePose)),
                        false)
                    .until(autoCommands::hasCoral)),

        // SCORE L4 ON K
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_K)
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            SCORING_CONSTRAINTS,
                            new AutoPoint(
                                new Pose2d(2.636, 6.497, Rotation2d.fromDegrees(-46.723)),
                                new AutoConstraintOptions(2.3, 57, 4, 30)),
                            new AutoPoint(
                                new Pose2d(3.266, 5.963, Rotation2d.fromDegrees(-46.723)),
                                new AutoConstraintOptions(1.5, 57, 4, 30)),
                            // REEF PIPE K
                            new AutoPoint(
                                robotManager.autoAlign::getUsedScoringPose,
                                new AutoConstraintOptions(1.5, 57, 4, 30))),
                        false)
                    .until(autoCommands::hasCoral)),
        autoCommands.l4ScoreAndReleaseCommand(),
        autoCommands
            .waitThenStow()
            .alongWith(

                // INTAKE STATION
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            INTAKING_CONSTRAINTS,
                            new AutoPoint(
                                new Pose2d(3.266, 5.963, Rotation2d.fromDegrees(-46.723))),
                            new AutoPoint(
                                new Pose2d(2.467, 6.611, Rotation2d.fromDegrees(132.723)),
                                autoCommands.intakeStationWarmupCommand(),
                                new AutoConstraintOptions(3, 57, 4, 30)),
                            new AutoPoint(Points.LEFT_CORAL_STATION.bluePose)),
                        false)
                    .until(autoCommands::hasCoral)),

        // SCORE L4 ON L
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_L)
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            SCORING_CONSTRAINTS,
                            new AutoPoint(
                                new Pose2d(2.636, 6.249, Rotation2d.fromDegrees(-46.723)),
                                new AutoConstraintOptions(2.3, 57, 4, 30)),
                            new AutoPoint(
                                new Pose2d(3.266, 5.615, Rotation2d.fromDegrees(-45.069)),
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
                new AutoPoint(new Pose2d(3.552, 5.238, Rotation2d.fromDegrees(-45.069))),
                new AutoPoint(
                    new Pose2d(3.266, 5.615, Rotation2d.fromDegrees(-45.069)),
                    autoCommands.stowRequest()))));
  }
}
