package frc.robot.robot_manager.collision_avoidance;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.ArrayList;
import java.util.Optional;

public class CollisionAvoidance {
  private static final double wristLength = 22.0;
  private static final SuperstructurePosition safePoint1 = new SuperstructurePosition(2.0, 75.0);
  private static final SuperstructurePosition safePoint2 = new SuperstructurePosition(0.0, 0.0);
  private static final SuperstructurePosition[] corners =
      new SuperstructurePosition[] {
        new SuperstructurePosition(2.0, 135.0), new SuperstructurePosition(65.0, 85.0)
      }; // TOP RIGHT CORNER IS 0 and TOP RIGHT CORNER IS 1
  private static ArrayList<SuperstructurePosition> possibleGoalPoints =
      new ArrayList<SuperstructurePosition>();
  private static double closestDistance;
  private static ArrayList<SuperstructurePosition> availablePoints =
      new ArrayList<SuperstructurePosition>();
  private static Translation2d goalPose = new Translation2d();

  public static Optional<SuperstructurePosition> plan(
      SuperstructurePosition current, SuperstructurePosition goal) {
    return plan(
        current.elevatorHeight(), current.wristAngle(), goal.elevatorHeight(), goal.wristAngle());
  }

  public static Optional<SuperstructurePosition> plan(
      double elevatorHeight, double wristAngle, double elevatorGoal, double wristGoal) {
    possibleGoalPoints.set(0, new SuperstructurePosition(elevatorGoal, wristGoal));
    possibleGoalPoints.set(1, safePoint1);
    possibleGoalPoints.set(2, safePoint2);
    //  getGoalPoint(possibleGoalPoints, angleHeightToPose(wristAngle, elevatorHeight))

    return Optional.of(
        poseToSuperstructurePosition(
            getGoalPoint(possibleGoalPoints, angleHeightToPose(wristAngle, elevatorHeight))));
  }

  private CollisionAvoidance() {}

  private static boolean collides(Translation2d currentPose, Translation2d goalPose) {

    double x1 = angleHeightToPose(corners[0].wristAngle(), corners[0].wristAngle()).getX();
    double y1 = angleHeightToPose(corners[0].wristAngle(), corners[0].wristAngle()).getY();
    double y2 = angleHeightToPose(corners[1].wristAngle(), corners[1].wristAngle()).getY();
    double x2 = angleHeightToPose(corners[1].wristAngle(), corners[1].wristAngle()).getX();

    double currentPoseX = currentPose.getX();
    double currentPoseY = currentPose.getY();

    double goalPoseX = goalPose.getX();
    double goalPoseY = goalPose.getY();
    // If the points are (x1, y1) and (x2, y2), then the two-point form reads:

    // y - y1 = (y2 - y1)/(x2 - x1) × (x - x1).
    Translation2d xInterceptionPoint =
        new Translation2d(
            x1,
            (goalPoseY - currentPoseY) / (goalPoseX - currentPoseX) * (x1 - currentPoseX)
                + currentPoseY);
    Translation2d yInterceptionPoint =
        new Translation2d(
            y1,
            (y1 - currentPoseY + currentPoseX)
                / ((goalPoseY - currentPoseY) / (goalPoseX - currentPoseX)));
    Translation2d y2InterceptionPoint =
        new Translation2d(
            y2,
            (y2 - currentPoseY + currentPoseX)
                / ((goalPoseY - currentPoseY) / (goalPoseX - currentPoseX)));

    if (y1 < xInterceptionPoint.getY() && xInterceptionPoint.getY() < y2) {
      return true;
    }
    if (x1 < yInterceptionPoint.getY() && yInterceptionPoint.getY() < x2) {
      return true;
    }
    if (x1 < y2InterceptionPoint.getY() && y2InterceptionPoint.getY() < x2) {
      return true;
    }
    return false;
  }

  private static Translation2d angleHeightToPose(double wristAngle, double elevatorHeight) {
    return new Translation2d(
        Math.cos(wristAngle) * wristLength, elevatorHeight + Math.sin(wristAngle) * wristLength);
  }

  private static SuperstructurePosition poseToSuperstructurePosition(Translation2d pose) {
    return new SuperstructurePosition(
        pose.getY() - (Math.sin(Math.cos(pose.getX() / wristLength)) * wristLength),
        Math.cos(pose.getX() / wristLength));
  }

  private static double distancefromPoses(Translation2d currentPose, Translation2d goalPose) {
    return Math.sqrt(
        (currentPose.getX() - goalPose.getX()) + (currentPose.getY() - goalPose.getY()));
  }

  private static Translation2d getGoalPoint(
      ArrayList<SuperstructurePosition> possibleGoalPoints, Translation2d currentPose) {
    for (int i = 0; i < possibleGoalPoints.size(); ) {
      if (!collides(
          currentPose,
          angleHeightToPose(
              possibleGoalPoints.get(i).wristAngle(),
              possibleGoalPoints.get(i).elevatorHeight()))) {
        availablePoints.set(i, possibleGoalPoints.get(i));
      }
      i++;
    }
    closestDistance = Double.MAX_VALUE;
    for (int w = 0; w < availablePoints.size(); w++) {

      if (distancefromPoses(
              currentPose,
              angleHeightToPose(
                  availablePoints.get(w).wristAngle(), availablePoints.get(w).elevatorHeight()))
          < closestDistance) {

        goalPose =
            angleHeightToPose(
                availablePoints.get(w).wristAngle(), availablePoints.get(w).elevatorHeight());
      }
    }
    return goalPose;
  }
}
