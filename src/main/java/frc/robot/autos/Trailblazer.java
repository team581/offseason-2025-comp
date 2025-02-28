package frc.robot.autos;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.constraints.AutoConstraintCalculator;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.autos.followers.PathFollower;
import frc.robot.autos.followers.PidPathFollower;
import frc.robot.autos.trackers.PathTracker;
import frc.robot.autos.trackers.pure_pursuit.PurePursuitPathTracker;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;

public class Trailblazer {
  /**
   * Given a point and the constraints for its parent segment, resolve the constraint options to use
   * while following that point.
   */
  private static AutoConstraintOptions resolveConstraints(
      AutoPoint point, AutoConstraintOptions segmentConstraints) {
    var constraints = point.constraints.orElse(segmentConstraints);
    return constraints;
  }

  private final SwerveSubsystem swerve;
  private final LocalizationSubsystem localization;
  private final PathTracker pathTracker = new PurePursuitPathTracker();
  private final PathFollower pathFollower =
      new PidPathFollower(
          new PIDController(5, 0, 0), new PIDController(5, 0, 0), new PIDController(12, 0, 0));
  private int previousAutoPointIndex = -1;
  private ChassisSpeeds previousSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private double previousTimestamp = 0.0;

  public Trailblazer(SwerveSubsystem swerve, LocalizationSubsystem localization) {
    this.swerve = swerve;
    this.localization = localization;
  }

  public Command followSegment(AutoSegment segment) {
    return followSegment(segment, true);
  }

  public Command followSegment(AutoSegment segment, boolean shouldEnd) {
    TrailblazerPathLogger.logSegment(segment);
    var command =
        Commands.runOnce(
                () -> {
                  pathTracker.resetAndSetPoints(segment.points);
                  previousAutoPointIndex = -1;
                  DogLog.log(
                      "Trailblazer/CurrentSegment/InitialPoints",
                      segment.points.stream()
                          .map(point -> point.poseSupplier.get())
                          .toArray(Pose2d[]::new));
                })
            .alongWith(
                Commands.run(
                    () -> {
                      pathTracker.updateRobotState(
                          localization.getPose(), swerve.getFieldRelativeSpeeds());
                      var currentAutoPointIndex = pathTracker.getCurrentPointIndex();
                      var currentAutoPoint = segment.points.get(currentAutoPointIndex);

                      var constrainedVelocityGoal =
                          getSwerveSetpoint(currentAutoPoint, segment.defaultConstraints);
                      swerve.setFieldRelativeAutoSpeeds(constrainedVelocityGoal);

                      DogLog.log("Trailblazer/Tracker/CurrentPointIndex", currentAutoPointIndex);
                      if (previousAutoPointIndex != currentAutoPointIndex) {
                        // Currently tracked point has changed, trigger side effects

                        // Each of the points in (previous, current]
                        var pointsToRunSideEffectsFor =
                            segment.points.subList(
                                previousAutoPointIndex + 1, currentAutoPointIndex + 1);
                        for (var passedPoint : pointsToRunSideEffectsFor) {
                          DogLog.log(
                              "Trailblazer/Tracker/CommandTriggered",
                              passedPoint.command.getName());
                          passedPoint.command.schedule();
                        }
                        previousAutoPointIndex = currentAutoPointIndex;
                      }
                    },
                    swerve))
            .withName("FollowSegmentIndefinitely");

    if (shouldEnd) {
      return command
          .until(pathTracker::isFinished)
          .andThen(
              Commands.runOnce(
                  () -> {
                    swerve.setFieldRelativeAutoSpeeds(new ChassisSpeeds());
                  }))
          .withName("FollowSegmentUntilFinished");
    }

    return command;
  }

  private ChassisSpeeds getSwerveSetpoint(
      AutoPoint point, AutoConstraintOptions segmentConstraints) {
    double currentTimestamp = Timer.getFPGATimestamp();
    if (previousTimestamp == 0.0) {
      previousTimestamp = currentTimestamp - 0.02;
    }
    var robotPose = localization.getPose();
    var originalTargetPose = pathTracker.getTargetPose();
    var originalVelocityGoal = pathFollower.calculateSpeeds(robotPose, originalTargetPose);
    var currentVelocity =
        Math.hypot(originalVelocityGoal.vxMetersPerSecond, originalVelocityGoal.vyMetersPerSecond);

    var usedConstraints = resolveConstraints(point, segmentConstraints);
    DogLog.log(
        "Trailblazer/Constraints/VelocityCalculation/CalculatedVelocity",
        usedConstraints.maxLinearVelocity());
    DogLog.log("Trailblazer/Tracker/RawOutput", originalTargetPose);

    DogLog.log("Trailblazer/Follower/RawOutput", originalVelocityGoal);
    var constrainedVelocityGoal =
        AutoConstraintCalculator.constrainVelocityGoal(
            originalVelocityGoal,
            previousSpeeds,
            currentTimestamp - previousTimestamp,
            usedConstraints);
    DogLog.log("Trailblazer/Follower/UsedOutput", constrainedVelocityGoal);

    previousTimestamp = currentTimestamp;
    previousSpeeds = constrainedVelocityGoal;

    return constrainedVelocityGoal;
  }
}
