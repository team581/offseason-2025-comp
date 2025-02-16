package frc.robot.autos.auto_path_commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.ReefPipe;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Trailblazer;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;

public class FourPiece2IJKLAuto extends BaseAuto {
  private static final AutoConstraintOptions CONSTRAINTS = new AutoConstraintOptions(2, 57, 4, 30);

  public FourPiece2IJKLAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getBlueStartingPose() {
    return Pose2d.kZero;
  }

  @Override
  protected Command getBlueAutoCommand() {
    return Commands.none();
  }

  @Override
  protected Pose2d getRedStartingPose() {
    return new Pose2d(10.289, 1.903, Rotation2d.kZero);
  }

  @Override
  protected Command getRedAutoCommand() {
    return Commands.sequence(
        // HOME AND PRELOAD
        actions.rehomeRollCommand(),
        autoCommands.preloadCoralCommand(),

        // SCORE L4 ON I
        trailblazer
            .followSegment(
                new AutoSegment(
                    CONSTRAINTS,
                    new AutoPoint(getRedStartingPose()),
                    new AutoPoint(
                        new Pose2d(11.785, 2.0, Rotation2d.fromDegrees(60)),
                        Commands.runOnce(
                            () -> {
                              AutoAlign.setAutoReefPipeOverride(ReefPipe.PIPE_I);
                              robotManager.l4CoralApproachRequest();
                            })),
                    new AutoPoint(() -> robotManager.purple.getUsedScoringPose())),
                false)
            .until(() -> robotManager.purple.isTagAligned()),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(13.701, 1.795, Rotation2d.fromDegrees(135.88))),
                new AutoPoint(
                    new Pose2d(16.01, 0.9, Rotation2d.fromDegrees(127.71)),
                    Commands.runOnce(() -> robotManager.intakeStationRequest())))),
        autoCommands.intakeStationWithTimeoutCommand(),

        // SCORE L4 ON J
        trailblazer
            .followSegment(
                new AutoSegment(
                    CONSTRAINTS,
                    new AutoPoint(new Pose2d(14.103, 1.0, Rotation2d.fromDegrees(90.5))),
                    new AutoPoint(
                        new Pose2d(12.246, 1.244, Rotation2d.fromDegrees(60.0)),
                        robotManager
                            .waitForStates(RobotState.IDLE_CORAL, RobotState.IDLE_NO_GP)
                            .andThen(
                                Commands.runOnce(
                                    () -> {
                                      AutoAlign.setAutoReefPipeOverride(ReefPipe.PIPE_J);
                                      robotManager.l4CoralApproachRequest();
                                    }))),
                    new AutoPoint(() -> robotManager.purple.getUsedScoringPose())),
                false)
            .until(() -> robotManager.purple.isTagAligned()),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(13.872, 1.903, Rotation2d.fromDegrees(135.88))),
                new AutoPoint(
                    new Pose2d(16.01, 0.9, Rotation2d.fromDegrees(127.71)),
                    Commands.runOnce(() -> robotManager.intakeStationRequest())))),
        autoCommands.intakeStationWithTimeoutCommand(),

        // SCORE L4 ON K
        robotManager
            .waitForStates(RobotState.IDLE_CORAL, RobotState.IDLE_NO_GP)
            .andThen(
                Commands.runOnce(
                    () -> {
                      AutoAlign.setAutoReefPipeOverride(ReefPipe.PIPE_K);
                      robotManager.l4CoralApproachRequest();
                    }))
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            CONSTRAINTS,
                            new AutoPoint(
                                new Pose2d(14.506, 1.903, Rotation2d.fromDegrees(133.277))),
                            new AutoPoint(
                                // REEF PIPE K
                                () -> robotManager.purple.getUsedScoringPose())),
                        false)
                    .until(() -> robotManager.purple.isTagAligned())),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE STATION
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(14.506, 1.795, Rotation2d.fromDegrees(133.277))),
                new AutoPoint(
                    new Pose2d(16.01, 0.9, Rotation2d.fromDegrees(123.819)),
                    Commands.runOnce(() -> robotManager.intakeStationRequest())))),
        autoCommands.intakeStationWithTimeoutCommand(),

        // SCORE L4 ON L
        robotManager
            .waitForStates(RobotState.IDLE_CORAL, RobotState.IDLE_NO_GP)
            .andThen(
                Commands.runOnce(
                    () -> {
                      AutoAlign.setAutoReefPipeOverride(ReefPipe.PIPE_L);
                      robotManager.l4CoralApproachRequest();
                    }))
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            CONSTRAINTS,
                            new AutoPoint(
                                new Pose2d(14.954, 1.971, Rotation2d.fromDegrees(134.931))),
                            // REEF PIPE L
                            new AutoPoint(() -> robotManager.purple.getUsedScoringPose())),
                        false)
                    .until(() -> robotManager.purple.isTagAligned())),
        autoCommands.l4ScoreAndReleaseCommand());
  }
}
