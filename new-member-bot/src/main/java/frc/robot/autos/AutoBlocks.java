package frc.robot.autos;

import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.AutoSegment;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.constraints.AutoConstraintOptions;
import com.team581.util.FmsUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import java.util.Optional;

public class AutoBlocks {

  private static final PoseErrorTolerance AFTER_SCORE_POSITION_TOLERANCE =
      new PoseErrorTolerance(0.8, 25);

  public static final PoseErrorTolerance APPROACH_REEF_TOLERANCE = new PoseErrorTolerance(0.6, 10);

  public static final AutoConstraintOptions MAX_CONSTRAINTS =
      new AutoConstraintOptions(4.75, 57, 4.0, 30);
  public static final AutoConstraintOptions LOLLIPOP_RACE_CONSTRAINTS =
      MAX_CONSTRAINTS.withMaxLinearVelocity(5.5).withMaxLinearAcceleration(4.5);
  public static final AutoConstraintOptions BASE_CONSTRAINTS =
      new AutoConstraintOptions(4.0, 30, 2.5, 25);
  private static final AutoConstraintOptions SCORING_CONSTRAINTS =
      BASE_CONSTRAINTS.withMaxLinearVelocity(1.5).withMaxLinearAcceleration(1.75);

  private final Trailblazer trailblazer;
  private final RobotManager robotManager;
  private final AutoCommands autoCommands;

  public AutoBlocks(Trailblazer trailblazer, RobotManager robotManager, AutoCommands autoCommands) {
    this.trailblazer = trailblazer;
    this.robotManager = robotManager;
    this.autoCommands = autoCommands;
  }

  public Command scorePreloadL1(Pose2d startingPose, ReefPipe pipe) {
    return Commands.sequence(
        trailblazer.followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                new AutoPoint(
                    () -> pipe.getPose(ReefPipeLevel.RAISING),
                    autoCommands
                        .preloadCoralCommand()
                        .andThen(autoCommands.l1ApproachCommand(pipe)),
                    BASE_CONSTRAINTS),
                new AutoPoint(() -> pipe.getPose(ReefPipeLevel.RAISING), BASE_CONSTRAINTS),
                new AutoPoint(
                    () -> pipe.getPose(ReefPipeLevel.RAISING),
                    autoCommands.l1ApproachCommand(pipe),
                    SCORING_CONSTRAINTS),
                // Actually align to score
                new AutoPoint(() -> pipe.getPose(ReefPipeLevel.L1), SCORING_CONSTRAINTS))),
        trailblazer.followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                AFTER_SCORE_POSITION_TOLERANCE,
                // Start at the scoring position
                new AutoPoint(
                    () -> pipe.getPose(ReefPipeLevel.RAISING),
                    Commands.waitSeconds(0.15).andThen(robotManager::stowRequest)),
                // Scoot back to the lineup position to finish the score
                new AutoPoint(() -> pipe.getPose(ReefPipeLevel.L1)))));
  }

  public Command scoreL1(ReefPipe pipe) {
    return scoreL1(Optional.empty(), pipe, Optional.empty());
  }

  public Command scoreL1(Pose2d approachPose, ReefPipe pipe) {
    return scoreL1(Optional.of(approachPose), pipe, Optional.empty());
  }

  public Command scoreL1(Pose2d approachPose, ReefPipe pipe, Pose2d backupPoint) {
    return scoreL1(Optional.of(approachPose), pipe, Optional.of(backupPoint));
  }

  public Command scoreL1(ReefPipe pipe, Pose2d backupPoint) {
    return scoreL1(Optional.empty(), pipe, Optional.of(backupPoint));
  }

  public Command scoreL1(
      Optional<Pose2d> approachPose, ReefPipe pipe, Optional<Pose2d> backupPoint) {
    var firstCommand =
        approachPose.isPresent()
            ? trailblazer
                .followSegment(
                    new AutoSegment(
                        SCORING_CONSTRAINTS,
                        new AutoPoint(approachPose.get()),
                        new AutoPoint(() -> pipe.getPose(ReefPipeLevel.L1))),
                    false)
                .withDeadline(autoCommands.waitForReleaseCommand().withTimeout(3))
            : trailblazer
                .followSegment(
                    new AutoSegment(
                        SCORING_CONSTRAINTS, new AutoPoint(() -> pipe.getPose(ReefPipeLevel.L1))),
                    false)
                .withDeadline(autoCommands.waitForReleaseCommand().withTimeout(3));
    return Commands.sequence(
        Commands.parallel(
            firstCommand,
            robotManager
                .waitForStates(
                    RobotState.CLAW_CORAL,
                    RobotState.CORAL_L1_APPROACH,
                    RobotState.STARTING_POSITION_CORAL)
                .andThen(autoCommands.l1ApproachCommand(pipe))),
        trailblazer
            .followSegment(
                new AutoSegment(
                    BASE_CONSTRAINTS,
                    new AutoPoint(
                        () ->
                            backupPoint.orElse(
                                pipe.getPose(
                                    ReefPipeLevel.BACK_AWAY, FmsUtil.isRedAlliance())))),
                false)
            .onlyIf(() -> robotManager.claw.getHasGP()));
  }
}
