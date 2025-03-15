package frc.robot.autos.trackers.pure_pursuit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.trackers.PathTracker;
import frc.robot.config.FeatureFlags;
import java.util.List;

public class PurePursuitPathTracker implements PathTracker {
  private static final double NON_DYNAMIC_LOOKAHEAD_DISTANCE = 1.5;
  private static final double DYNAMIC_LOOKAHEAD_TRANSITION_TIME = 0.5;

  private static final double AT_END_OF_SEGMENT_DISTANCE_THRESHOLD = 0.1;
  private static final double AT_END_OF_SEGMENT_ROTATION_THRESHOLD = 5;

  private double lookaheadDistance = 1.5;
  private double lastRequestedLookaheadDistance = Double.MAX_VALUE;
  private double transitionStartTime = 0.0;
  private double lastStartTime = 0.0;

  private List<AutoPoint> points = List.of();
  private Pose2d currentRobotPose = Pose2d.kZero;
  private Pose2d startingRobotPose = Pose2d.kZero;
  private boolean startingRobotPoseUpdated = false;
  private Pose2d lastTargetWaypoint = Pose2d.kZero;
  private Pose2d currentTargetWaypoint = Pose2d.kZero;
  private int currentLookaheadPointIndex = 0;
  private int currentRobotFollowedPointIndex = 0;
  private Rotation2d currentInterpolatedRotation = Rotation2d.kZero;

  @Override
  public void resetAndSetPoints(List<AutoPoint> points) {
    startingRobotPose = Pose2d.kZero;
    startingRobotPoseUpdated = false;
    currentLookaheadPointIndex = 0;
    currentRobotFollowedPointIndex = 0;
    this.points = points;
  }

  @Override
  public void updateRobotState(Pose2d currentPose, ChassisSpeeds currentFieldRelativeRobotSpeeds) {
    this.currentRobotPose = currentPose;

    if (!startingRobotPoseUpdated) {
      startingRobotPose = currentPose;
      startingRobotPoseUpdated = true;
    }
  }

  @Override
  public Pose2d getTargetPose() {
    updateLookahead();
    updateRotation();
    if (points.isEmpty()) {
      return Pose2d.kZero;
    }
    if (getCurrentLookaheadPointIndex() == 0) {
      lastTargetWaypoint = startingRobotPose;
    } else {
      lastTargetWaypoint = points.get(getCurrentLookaheadPointIndex() - 1).poseSupplier.get();
    }
    currentTargetWaypoint = points.get(getCurrentLookaheadPointIndex()).poseSupplier.get();
    var perpendicularPoint =
        PurePursuitUtils.getPerpendicularPoint(
            lastTargetWaypoint, currentTargetWaypoint, currentRobotPose);

    var lookaheadPoint =
        new Pose2d(
            PurePursuitUtils.getLookaheadPoint(
                    lastTargetWaypoint,
                    currentTargetWaypoint,
                    perpendicularPoint,
                    lookaheadDistance
                        - currentRobotPose
                            .getTranslation()
                            .getDistance(perpendicularPoint.getTranslation()))
                .getTranslation(),
            currentInterpolatedRotation);

    var lookaheadInside =
        PurePursuitUtils.isBetween(lastTargetWaypoint, currentTargetWaypoint, lookaheadPoint);
    var lookaheadToStartDistance =
        lookaheadPoint.getTranslation().getDistance(lastTargetWaypoint.getTranslation());
    var lookaheadToEndDistance =
        lookaheadPoint.getTranslation().getDistance(currentTargetWaypoint.getTranslation());
    if (!lookaheadInside) {
      if (lookaheadToEndDistance > lookaheadToStartDistance) {
        return new Pose2d(lastTargetWaypoint.getTranslation(), currentInterpolatedRotation);
      }
      if (getCurrentLookaheadPointIndex() < points.size() - 1) {
        var futurePoint = points.get(getCurrentLookaheadPointIndex() + 1).poseSupplier.get();
        var perpendicularToCurrentEndDistance =
            perpendicularPoint.getTranslation().getDistance(currentTargetWaypoint.getTranslation());
        var newLookaheadPoint =
            new Pose2d(
                PurePursuitUtils.getLookaheadPoint(
                        currentTargetWaypoint,
                        futurePoint,
                        currentTargetWaypoint,
                        lookaheadDistance - perpendicularToCurrentEndDistance)
                    .getTranslation(),
                currentInterpolatedRotation);

        currentLookaheadPointIndex++;
        var newLookaheadInside =
            PurePursuitUtils.isBetween(currentTargetWaypoint, futurePoint, newLookaheadPoint);
        if (!newLookaheadInside) {
          return new Pose2d(currentTargetWaypoint.getTranslation(), currentInterpolatedRotation);
        }

        return newLookaheadPoint;
      } else {
        return new Pose2d(currentTargetWaypoint.getTranslation(), currentInterpolatedRotation);
      }
    }
    return lookaheadPoint;
  }

  private void updateRotation() {
    var lastTargetWaypoint = Pose2d.kZero;
    if (currentRobotFollowedPointIndex == 0) {
      lastTargetWaypoint = startingRobotPose;
    } else {
      lastTargetWaypoint = points.get(currentRobotFollowedPointIndex - 1).poseSupplier.get();
    }
    var currentTargetPoint = points.get(currentRobotFollowedPointIndex).poseSupplier.get();

    var perpendicularPoint =
        PurePursuitUtils.getPerpendicularPoint(
            lastTargetWaypoint, currentTargetPoint, currentRobotPose);
    if (currentRobotFollowedPointIndex < points.size() - 1
        && perpendicularPoint.getTranslation().getDistance(lastTargetWaypoint.getTranslation())
                / lastTargetWaypoint
                    .getTranslation()
                    .getDistance(currentTargetPoint.getTranslation())
            > 0.9) {

      currentRobotFollowedPointIndex++;
      currentTargetWaypoint = points.get(currentRobotFollowedPointIndex).poseSupplier.get();
      lastTargetWaypoint = points.get(currentRobotFollowedPointIndex - 1).poseSupplier.get();
    }

    if (FeatureFlags.PURE_PURSUIT_ROTATE_IMMEDIATELY.getAsBoolean()) {
      currentInterpolatedRotation = currentTargetPoint.getRotation();
    } else {
      currentInterpolatedRotation =
          PurePursuitUtils.getPointToPointInterpolatedRotation(
              lastTargetWaypoint, currentTargetPoint, perpendicularPoint);
    }
  }

  private int getCurrentLookaheadPointIndex() {
    return currentLookaheadPointIndex;
  }

  @Override
  public int getCurrentPointIndex() {
    return currentRobotFollowedPointIndex;
  }

  private void updateLookahead() {
    if (FeatureFlags.PURE_PURSUIT_USE_DYNAMIC_LOOKAHEAD.getAsBoolean() && points.size() > 1) {

      if (points.size() == 2) {
        requestNewLookaheadDistance(
            PurePursuitUtils.getDynamicLookaheadDistance(
                startingRobotPose,
                points.get(0).poseSupplier.get(),
                points.get(1).poseSupplier.get()),
            false);
      } else if (getCurrentPointIndex() == points.size() - 1) {
        requestNewLookaheadDistance(
            PurePursuitUtils.getDynamicLookaheadDistance(
                points.get(getCurrentPointIndex() - 2).poseSupplier.get(),
                lastTargetWaypoint,
                currentTargetWaypoint),
            false);
      } else {
        var thirdPoint = points.get(getCurrentPointIndex() + 1).poseSupplier.get();
        requestNewLookaheadDistance(
            PurePursuitUtils.getDynamicLookaheadDistance(
                lastTargetWaypoint, currentTargetWaypoint, thirdPoint),
            false);
      }
    } else {
      requestNewLookaheadDistance(NON_DYNAMIC_LOOKAHEAD_DISTANCE, true);
    }
  }

  public void requestNewLookaheadDistance(double targetLookahead, boolean immediateChnage) {
    double currentTime = Timer.getFPGATimestamp();
    if (lastRequestedLookaheadDistance != targetLookahead) {
      transitionStartTime = currentTime;
    } else {
      transitionStartTime = lastStartTime;
    }
    if ((currentTime < transitionStartTime + DYNAMIC_LOOKAHEAD_TRANSITION_TIME)
        && !immediateChnage) {
      // Calculate the progress of the transition
      double progress = (currentTime - transitionStartTime) / DYNAMIC_LOOKAHEAD_TRANSITION_TIME;

      // Linear interpolation (you can use other easing functions here)
      double smoothLookahead = lookaheadDistance + (targetLookahead - lookaheadDistance) * progress;
      lastRequestedLookaheadDistance = targetLookahead;
      lastStartTime = transitionStartTime;
      lookaheadDistance = smoothLookahead;
    } else {
      // Transition completed, return target lookahead
      lastRequestedLookaheadDistance = targetLookahead;
      lastStartTime = transitionStartTime;
      lookaheadDistance = targetLookahead;
    }
  }
}
