package frc.robot.util.trailblazer;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.TrailblazerPathLogger;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.trailblazer.followers.PathFollower;
import frc.robot.util.trailblazer.followers.ProfiledPidPathFollower;
import frc.robot.util.trailblazer.trackers.PathTracker;
import frc.robot.util.trailblazer.trackers.pure_pursuit.PurePursuitPathTracker;

public class Trailblazer {
  private final SwerveSubsystem swerve;
  private final LocalizationSubsystem localization;
  private final PathTracker pathTracker = new PurePursuitPathTracker();
  private final PathFollower pathFollower =
      new ProfiledPidPathFollower(new PIDController(7, 0, 0), new PIDController(7, 0, 0));
  private int previousAutoPointIndex = -1;

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
                      "Autos/Trailblazer/CurrentSegment/InitialPoints",
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

                      var constrainedVelocityGoal = getSwerveSetpoint(currentAutoPoint, segment);
                      swerve.setFieldRelativeAutoSpeeds(constrainedVelocityGoal);

                      DogLog.log("Autos/Trailblazer/Tracker/Output", pathTracker.getTargetPose());
                      DogLog.log("Autos/Trailblazer/Follower/Output", constrainedVelocityGoal);

                      DogLog.log(
                          "Autos/Trailblazer/UsedConstraints/MaxLinearV",
                          segment.getConstraints(currentAutoPoint).linearConstraints().maxVelocity);
                      DogLog.log(
                          "Autos/Trailblazer/UsedConstraints/MaxLinearA",
                          segment
                              .getConstraints(currentAutoPoint)
                              .linearConstraints()
                              .maxAcceleration);
                      DogLog.log(
                          "Autos/Trailblazer/UsedConstraints/MaxAngularV",
                          segment
                              .getConstraints(currentAutoPoint)
                              .angularConstraints()
                              .maxVelocity);
                      DogLog.log(
                          "Autos/Trailblazer/UsedConstraints/MaxAngularA",
                          segment
                              .getConstraints(currentAutoPoint)
                              .angularConstraints()
                              .maxAcceleration);

                      DogLog.log(
                          "Autos/Trailblazer/Tracker/CurrentPointIndex", currentAutoPointIndex);
                      if (previousAutoPointIndex != currentAutoPointIndex) {
                        // Currently tracked point has changed, trigger side effects

                        // Each of the points in (previous, current]
                        var pointsToRunSideEffectsFor =
                            segment.points.subList(
                                previousAutoPointIndex + 1, currentAutoPointIndex + 1);
                        for (var passedPoint : pointsToRunSideEffectsFor) {
                          DogLog.log(
                              "Autos/Trailblazer/Tracker/CommandTriggered",
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
          .until(
              () -> segment.isFinished(localization.getPose(), pathTracker.getCurrentPointIndex()))
          .andThen(Commands.runOnce(() -> swerve.setFieldRelativeAutoSpeeds(new ChassisSpeeds())))
          .withName("FollowSegmentUntilFinished");
    }

    return command;
  }

  private ChassisSpeeds getSwerveSetpoint(AutoPoint point, AutoSegment segment) {
    var lastPoint = segment.points.get(segment.points.size() - 1);
    var robotPose = localization.getPose();
    var goalPose = pathTracker.getTargetPose();
    var distanceToSegmentEnd =
        segment.getRemainingDistance(localization.getPose(), pathTracker.getCurrentPointIndex());

    // TODO: Calling this calculate speeds function has side effects, need a better method for
    // deciding if we are going to overshoot
    // var speedsForRemainingPath =
    //     pathFollower.calculateSpeeds(
    //         Pose2d.kZero,
    //         new Pose2d(distanceToSegmentEnd, 0, Rotation2d.kZero),
    //         segment.getConstraints(lastPoint));
    var speedsForCurrentPoint =
        pathFollower.calculateSpeeds(robotPose, goalPose, segment.getConstraints(point));

    // speedsForCurrentPoint.vMetersPerSecond = Math.min(speedsForRemainingPath.vMetersPerSecond,
    // speedsForCurrentPoint.vMetersPerSecond);

    return speedsForCurrentPoint;
  }
}
