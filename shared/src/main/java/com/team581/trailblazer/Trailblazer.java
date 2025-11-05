package com.team581.trailblazer;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.team581.math.TimestampedChassisSpeeds;
import com.team581.trailblazer.constraints.AutoConstraintCalculator;
import com.team581.trailblazer.constraints.AutoConstraintOptions;
import com.team581.trailblazer.followers.PathFollower;
import com.team581.trailblazer.trackers.PathTracker;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Trailblazer is Team 581's custom path following library. We built Trailblazer to give us end to
 * end control over every aspect of how our autos execute.
 *
 * <p>Trailblazer is made up of a few components:
 *
 * <ol>
 *   <li>Path segments, which are a list of points to follow with the robot
 *   <li>Can include constraints on robot motion per point or per segment
 *   <li>Commands can be attached to points to run side effects
 *   <li>Path trackers, which determine the pose setpoint for the robot
 *   <li>Path followers, which calculate a velocity setpoint to reach the pose setpoint
 * </ol>
 *
 * Trailblazer paths are executed by the {@link #followSegment} method, which returns a command that
 * can be composed in autos.
 */
public class Trailblazer {
  /**
   * Given a point and the constraints for its parent segment, resolve the constraint options to use
   * while following that point.
   */
  private static AutoConstraintOptions resolveConstraints(
      AutoPoint point, AutoConstraintOptions segmentConstraints) {
    return point.constraints.orElse(segmentConstraints);
  }

  private final SwerveBase swerve;
  private final LocalizationBase localization;

  private final PathTracker pathTracker;
  private final PathFollower pathFollower;
  private int previousAutoPointIndex = -1;
  private TimestampedChassisSpeeds previousSpeeds = new TimestampedChassisSpeeds(0);

  public Trailblazer(
      SwerveBase swerve,
      LocalizationBase localization,
      PathTracker pathTracker,
      PathFollower pathFollower) {
    this.swerve = swerve;
    this.localization = localization;
    this.pathTracker = pathTracker;
    this.pathFollower = pathFollower;
  }

  public Command followSegment(AutoSegment segment) {
    return followSegment(segment, true);
  }

  public Command followSegment(AutoSegment segment, boolean shouldEnd) {
    TrailblazerPathLogger.logSegment(segment);
    var command =
        Commands.runOnce(() -> followSegmentInit(segment))
            .alongWith(Commands.run(() -> followSegmentPeriodic(segment), swerve))
            .withName("FollowSegmentIndefinitely");

    if (shouldEnd) {
      return command
          .until(() -> followSegmentIsFinished(segment))
          .andThen(
              Commands.runOnce(
                  () -> {
                    swerve.setFieldRelativeAutoSpeeds(new ChassisSpeeds());
                  }))
          .withName("FollowSegmentUntilFinished");
    }

    return command;
  }

  public boolean followSegmentIsFinished(AutoSegment segment) {
    return segment.isFinished(localization.getPose(), pathTracker.getCurrentPointIndex());
  }

  public void followSegmentPeriodic(AutoSegment segment) {
    pathTracker.updateRobotState(localization.getPose(), swerve.getFieldRelativeSpeeds());
    var currentAutoPointIndex = pathTracker.getCurrentPointIndex();
    var currentAutoPoint = segment.points.get(currentAutoPointIndex);
    double distanceToSegmentEnd =
        segment.getRemainingDistance(localization.getPose(), currentAutoPointIndex);

    var constrainedVelocityGoal =
        getSwerveSetpoint(currentAutoPoint, segment.defaultConstraints, distanceToSegmentEnd);
    swerve.setFieldRelativeAutoSpeeds(constrainedVelocityGoal);

    DogLog.log("Autos/Trailblazer/Tracker/CurrentPointIndex", currentAutoPointIndex);
    if (previousAutoPointIndex != currentAutoPointIndex) {
      // Currently tracked point has changed, trigger side effects

      // Each of the points in (previous, current]
      var pointsToRunSideEffectsFor =
          segment.points.subList(previousAutoPointIndex + 1, currentAutoPointIndex + 1);
      for (var passedPoint : pointsToRunSideEffectsFor) {
        DogLog.log("Autos/Trailblazer/Tracker/CommandTriggered", passedPoint.command.getName());
        passedPoint.command.schedule();
      }
      previousAutoPointIndex = currentAutoPointIndex;
    }
  }

  public void followSegmentInit(AutoSegment segment) {
    pathTracker.resetAndSetPoints(segment.points);
    previousAutoPointIndex = -1;
    DogLog.log(
        "Autos/Trailblazer/CurrentSegment/InitialPoints",
        segment.points.stream().map(point -> point.poseSupplier.get()).toArray(Pose2d[]::new));
  }

  private ChassisSpeeds getSwerveSetpoint(
      AutoPoint point, AutoConstraintOptions segmentConstraints, double distanceToSegmentEnd) {
    if (previousSpeeds.timestampSeconds == 0) {
      previousSpeeds = new TimestampedChassisSpeeds(Timer.getFPGATimestamp() - 0.02);
    }

    var robotPose = localization.getPose();
    var originalTargetPose = pathTracker.getTargetPose();
    var originalVelocityGoal =
        new TimestampedChassisSpeeds(pathFollower.calculateSpeeds(robotPose, originalTargetPose));
    var originalConstraints = resolveConstraints(point, segmentConstraints);

    /*
    var newLinearVelocity =
        AutoConstraintCalculator.getDynamicVelocityConstraint(
            robotPose,
            endPose,
            swerve.getFieldRelativeSpeeds(),
            originalConstraints.maxLinearVelocity(),
            originalConstraints.maxLinearAcceleration());
    */
    var usedConstraints =
        originalConstraints.withMaxLinearVelocity(originalConstraints.maxLinearVelocity());

    DogLog.log(
        "Autos/Trailblazer/Constraints/VelocityCalculation/CalculatedLinearVelocity",
        usedConstraints.maxLinearVelocity(),
        MetersPerSecond);
    DogLog.log(
        "Autos/Trailblazer/Constraints/Acceleration/CalulatedLinearAcceleration",
        usedConstraints.maxLinearAcceleration(),
        MetersPerSecondPerSecond);
    DogLog.log("Autos/Trailblazer/Tracker/RawOutput", originalTargetPose);

    DogLog.log("Autos/Trailblazer/Follower/RawOutput", originalVelocityGoal);
    var constrainedVelocityGoal =
        AutoConstraintCalculator.constrainVelocityGoal(
            originalVelocityGoal, previousSpeeds, usedConstraints, distanceToSegmentEnd);
    DogLog.log("Autos/Trailblazer/Follower/UsedOutput", constrainedVelocityGoal);

    previousSpeeds = constrainedVelocityGoal;

    return constrainedVelocityGoal;
  }
}
