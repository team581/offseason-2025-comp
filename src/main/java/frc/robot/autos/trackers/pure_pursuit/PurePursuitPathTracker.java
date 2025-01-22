package frc.robot.autos.trackers.pure_pursuit;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.trackers.PathTracker;
import java.util.List;

public class PurePursuitPathTracker implements PathTracker {
  private static final boolean USE_DYNAMIC_LOOKAHEAD = true;
  private static final double NON_DYNAMIC_LOOKAHEAD_DISTANCE = 1.5;
  private static final double AT_END_OF_SEGMENT_DISTANCE_THRESHOLD = 0.1;
  private static final double AT_END_OF_SEGMENT_ROTATION_THRESHOLD = 5;
  private static final double DYNAMIC_LOOKAHEAD_TRANSITION_TIME = 0.5;
  private static final double DYNAMIC_LOOKAHEAD_SCALE = 0.45;
  private static final double DYNAMIC_LOOKAHEAD_MAX = 2.0;
  private double lookaheadDistance = 1.5;
  private double lastRequestedLookaheadDistance = Double.MAX_VALUE;
  private double transitionStartTime = 0.0;
  private double lastStartTime = 0.0;

  private List<AutoPoint> points = List.of();
  private Pose2d currentRobotPose = new Pose2d();
  private Pose2d startingRobotPose = new Pose2d();
  private boolean startingRobotPoseUpdated = false;
  private Pose2d lastTargetWaypoint = new Pose2d();
  private Pose2d currentTargetWaypoint = new Pose2d();
  private int currentLookaheadPointIndex = 0;
  private int currentRobotFollowedPointIndex = 0;
  private Rotation2d currentInterpolatedRotation = new Rotation2d();

  @Override
  public void resetAndSetPoints(List<AutoPoint> points) {
    startingRobotPose = new Pose2d();
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
    DogLog.log("Autos/PurePursuitPathTracker/CurrentPointIndex", getCurrentPointIndex());

    DogLog.log(
        "Autos/PurePursuitPathTracker/CurrentLookaheadPointIndex", getCurrentLookaheadPointIndex());
    DogLog.log("Autos/PurePursuitPathTracker/StartingRobotPose/Point", startingRobotPose);
    DogLog.log("Autos/PurePursuitPathTracker/StartingRobotPose/Updated", startingRobotPoseUpdated);
    DogLog.log("Autos/PurePursuitPathTracker/LookaheadDistance", lookaheadDistance);
    DogLog.log("Autos/PurePursuitPathTracker/Waypoints/Start", lastTargetWaypoint);
    DogLog.log("Autos/PurePursuitPathTracker/Waypoints/End", currentTargetWaypoint);
  }

  @Override
  public Pose2d getTargetPose() {
    DogLog.log("Autos/PurePursuitPathTracker/Size", points.size());
    updateLookahead();
    updateRotation();
    DogLog.log("Autos/PurePursuitPathTracker/Size", points.size());
    if (points.isEmpty()) {
      return new Pose2d();
    }
    if (getCurrentLookaheadPointIndex() == 0) {
      lastTargetWaypoint = startingRobotPose;
    } else {
      lastTargetWaypoint = points.get(getCurrentLookaheadPointIndex() - 1).poseSupplier.get();
    }
    currentTargetWaypoint = points.get(getCurrentLookaheadPointIndex()).poseSupplier.get();
    var perpendicularPoint =
        getPerpendicularPoint(lastTargetWaypoint, currentTargetWaypoint, currentRobotPose);

    var lookaheadPoint =
        new Pose2d(
            getLookaheadPoint(
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
                getLookaheadPoint(
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
    var lastTargetWaypoint = new Pose2d();
    if (currentRobotFollowedPointIndex == 0) {
      lastTargetWaypoint = startingRobotPose;
    } else {
      lastTargetWaypoint = points.get(currentRobotFollowedPointIndex - 1).poseSupplier.get();
    }
    var currentTargetPoint = points.get(currentRobotFollowedPointIndex).poseSupplier.get();

    var perpendicularPoint =
        getPerpendicularPoint(lastTargetWaypoint, currentTargetPoint, currentRobotPose);
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
    currentInterpolatedRotation =
        getPointToPointInterpolatedRotation(
            lastTargetWaypoint, currentTargetPoint, perpendicularPoint);
  }

  private int getCurrentLookaheadPointIndex() {
    return currentLookaheadPointIndex;
  }

  @Override
  public int getCurrentPointIndex() {
    return currentRobotFollowedPointIndex;
  }

  @Override
  public boolean isFinished() {
    if (points.isEmpty()) {
      return true;
    }
    if ((currentRobotPose
                .getTranslation()
                .getDistance(points.get(points.size() - 1).poseSupplier.get().getTranslation())
            < AT_END_OF_SEGMENT_DISTANCE_THRESHOLD)
        && MathUtil.isNear(
            points.get(points.size() - 1).poseSupplier.get().getRotation().getDegrees(),
            currentRobotPose.getRotation().getDegrees(),
            AT_END_OF_SEGMENT_ROTATION_THRESHOLD)) {
      return true;
    }
    return false;
  }

  private void updateLookahead() {
    if (USE_DYNAMIC_LOOKAHEAD && points.size() > 1) {

      if (points.size() == 2) {
        requestNewLookaheadDistance(
            getDynamicLookaheadDistance(
                startingRobotPose,
                points.get(0).poseSupplier.get(),
                points.get(1).poseSupplier.get()),
            false);
      } else if (getCurrentPointIndex() == points.size() - 1) {
        requestNewLookaheadDistance(
            getDynamicLookaheadDistance(
                points.get(getCurrentPointIndex() - 2).poseSupplier.get(),
                lastTargetWaypoint,
                currentTargetWaypoint),
            false);
      } else {
        var thirdPoint = points.get(getCurrentPointIndex() + 1).poseSupplier.get();
        requestNewLookaheadDistance(
            getDynamicLookaheadDistance(lastTargetWaypoint, currentTargetWaypoint, thirdPoint),
            false);
      }
    } else {
      requestNewLookaheadDistance(NON_DYNAMIC_LOOKAHEAD_DISTANCE, true);
    }
  }

  private Rotation2d getPointToPointInterpolatedRotation(
      Pose2d startPoint, Pose2d endPoint, Pose2d pointOnPath) {
    var totalDistance = startPoint.getTranslation().getDistance(endPoint.getTranslation());
    var pointToStart = pointOnPath.getTranslation().getDistance(startPoint.getTranslation());
    var pointToEnd = pointOnPath.getTranslation().getDistance(endPoint.getTranslation());

    if (!((pointOnPath.getX() - startPoint.getX()) * (pointOnPath.getX() - endPoint.getX()) <= 0
        && (pointOnPath.getY() - startPoint.getY()) * (pointOnPath.getY() - endPoint.getY())
            <= 0)) {
      if (pointToEnd > pointToStart) {
        return startPoint.getRotation();
      } else {
        return endPoint.getRotation();
      }
    }
    var progressPercent = Math.abs((pointToStart / totalDistance));
    if (progressPercent > 0.9) {
      progressPercent = 1.0;
    }

    var interpolatedRotation =
        startPoint.getRotation().interpolate(endPoint.getRotation(), progressPercent);

    return interpolatedRotation;
  }

  private double getDynamicLookaheadDistance(
      Pose2d firstPoint, Pose2d secondPoint, Pose2d thirdPoint) {
    var x1 = firstPoint.getX();
    var y1 = firstPoint.getY();
    var x2 = secondPoint.getX();
    var y2 = secondPoint.getY();
    var x3 = thirdPoint.getX();
    var y3 = thirdPoint.getY();

    var AB = Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    var BC = Math.sqrt((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    var AC = Math.sqrt((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
    var s = (AB + BC + AC) / 2;
    var area = Math.sqrt(s * (s - AB) * (s - BC) * (s - AC));
    if (area == 0) {
      return DYNAMIC_LOOKAHEAD_MAX;
    }
    var curvature = (AB * BC * AC) / (4 * area);
    double value = curvature * DYNAMIC_LOOKAHEAD_SCALE;
    Pose2d[] curvaturepoints = {firstPoint, secondPoint, thirdPoint};
    DogLog.log("Autos/Trailblazer/PurePursuitPathTracker/CurvaturePoints", curvaturepoints);
    return Math.min(value, DYNAMIC_LOOKAHEAD_MAX);
  }

  private Pose2d getPerpendicularPoint(Pose2d startPoint, Pose2d endPoint, Pose2d robotPose) {
    var x1 = startPoint.getX();
    var y1 = startPoint.getY();

    var x2 = endPoint.getX();
    var y2 = endPoint.getY();

    var x3 = robotPose.getX();
    var y3 = robotPose.getY();

    if (y1 == y2) {
      return new Pose2d(x3, y1, new Rotation2d());
    }
    if (x1 == x2) {
      return new Pose2d(x1, y3, new Rotation2d());
    }
    // Find the slope of the path and y-int
    double pathSlope = (y2 - y1) / (x2 - x1);
    double yInt = y1 - pathSlope * x1;

    // Find the slope and y-int of the perpendicular line
    double perpSlope = -1 / pathSlope;
    double perpYInt = y3 - perpSlope * x3;

    // Calculate the perpendicular intersection
    double perpX = (perpYInt - yInt) / (pathSlope - perpSlope);
    double perpY = pathSlope * perpX + yInt;

    return new Pose2d(perpX, perpY, new Rotation2d());
  }

  private Pose2d getLookaheadPoint(
      Pose2d startPoint, Pose2d endPoint, Pose2d pointOnPath, double lookaheadDistance) {
    var x1 = startPoint.getX();
    var y1 = startPoint.getY();

    var x2 = endPoint.getX();
    var y2 = endPoint.getY();

    var x = pointOnPath.getX();
    var y = pointOnPath.getY();

    var xLookahead =
        x
            + lookaheadDistance
                * ((x2 - x1) / (Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2))));
    var yLookahead =
        y
            + lookaheadDistance
                * ((y2 - y1) / (Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2))));
    var lookahead = new Pose2d(xLookahead, yLookahead, new Rotation2d());
    var distanceToStart = lookahead.getTranslation().getDistance(startPoint.getTranslation());
    var distanceToEnd = lookahead.getTranslation().getDistance(endPoint.getTranslation());
    var lookaheadOutside =
        !((lookahead.getX() - startPoint.getX()) * (lookahead.getX() - endPoint.getX()) <= 0
            && (lookahead.getY() - startPoint.getY()) * (lookahead.getY() - endPoint.getY()) <= 0);
    if (lookaheadOutside) {
      if (distanceToEnd > distanceToStart) {
        return startPoint;
      }
    }
    return getPerpendicularPoint(startPoint, endPoint, lookahead);
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
