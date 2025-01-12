package frc.robot.robot_manager.collision_avoidance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.ArrayList;
import java.util.Optional;

public class CollisionAvoidance {
  private static final double wristLength = 0.0;
  private static final Pose2d safePoint1 = new Pose2d();
  private static final Pose2d safePoint2 = new Pose2d();
  private static final Pose2d[] corners =
      new Pose2d[] {
        new Pose2d(-1, 1, Rotation2d.fromDegrees(0)), new Pose2d(1, 2, Rotation2d.fromDegrees(0))
      }; // TOP RIGHT CORNER IS 0 and TOP RIGHT CORNER IS 1
  private static ArrayList<Pose2d> possibleGoalPoints = new ArrayList<Pose2d>();
  private static double closestDistance;
  private static ArrayList<Pose2d> availablePoints = new ArrayList<Pose2d>();
  private static Pose2d goalPose = new Pose2d();

  public static Optional<SuperstructurePosition> plan(
      SuperstructurePosition current, SuperstructurePosition goal) {
    return plan(
        current.elevatorHeight(), current.wristAngle(), goal.elevatorHeight(), goal.wristAngle());
  }

  public static Optional<SuperstructurePosition> plan(
      double elevatorHeight, double wristAngle, double elevatorGoal, double wristGoal) {
    possibleGoalPoints.set(0, angleHeightToPose(wristGoal, elevatorGoal));
    possibleGoalPoints.set(1, safePoint1);
    possibleGoalPoints.set(2, safePoint2);
    //  getGoalPoint(possibleGoalPoints, angleHeightToPose(wristAngle, elevatorHeight))

    return Optional.of(
        poseToSuperstructurePosition(
            getGoalPoint(possibleGoalPoints, angleHeightToPose(wristAngle, elevatorHeight))));
  }

  private CollisionAvoidance() {}

  private static boolean collides(Pose2d currentPose, Pose2d goalPose) {

    double x1 = corners[0].getX();
    double y1 = corners[0].getY();
    double y2 = corners[1].getY();
    double x2 = corners[1].getX();

    double currentPoseX = currentPose.getX();
    double currentPoseY = currentPose.getY();

    double goalPoseX = goalPose.getX();
    double goalPoseY = goalPose.getY();
    // If the points are (x1, y1) and (x2, y2), then the two-point form reads:

    // y - y1 = (y2 - y1)/(x2 - x1) × (x - x1).
    Pose2d xInterceptionPoint =
        new Pose2d(
            x1,
            (goalPoseY - currentPoseY) / (goalPoseX - currentPoseX) * (x1 - currentPoseX)
                + currentPoseY,
            Rotation2d.fromDegrees(0));
    Pose2d yInterceptionPoint =
        new Pose2d(
            y1,
            (y1 - currentPoseY + currentPoseX)
                / ((goalPoseY - currentPoseY) / (goalPoseX - currentPoseX)),
            Rotation2d.fromDegrees(0));
    Pose2d y2InterceptionPoint =
        new Pose2d(
            y2,
            (y2 - currentPoseY + currentPoseX)
                / ((goalPoseY - currentPoseY) / (goalPoseX - currentPoseX)),
            Rotation2d.fromDegrees(0));

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

  private static Pose2d angleHeightToPose(double wristAngle, double elevatorHeight) {
    return new Pose2d(
        Math.cos(wristAngle) * wristLength,
        elevatorHeight + Math.sin(wristAngle) * wristLength,
        Rotation2d.fromDegrees(0.0));
  }

  private static SuperstructurePosition poseToSuperstructurePosition(Pose2d pose) {
    return new SuperstructurePosition(
        pose.getY() - (Math.sin(Math.cos(pose.getX() / wristLength)) * wristLength),
        Math.cos(pose.getX() / wristLength));
  }

  private static double distancefromPoses(Pose2d currentPose, Pose2d goalPose) {
    return Math.sqrt(
        (currentPose.getX() - goalPose.getX()) + (currentPose.getY() - goalPose.getY()));
  }

  private static Pose2d getGoalPoint(ArrayList<Pose2d> possibleGoalPoints, Pose2d currentPose) {
    for (int i = 0; i < possibleGoalPoints.size(); ) {
      if (!collides(currentPose, possibleGoalPoints.get(i))) {
        availablePoints.set(i, possibleGoalPoints.get(i));
      }
      i++;
    }
    closestDistance = Double.MAX_VALUE;
    for (int w = 0; w < availablePoints.size(); w++) {

      if (distancefromPoses(currentPose, availablePoints.get(w)) < closestDistance) {

        goalPose = availablePoints.get(w);
      }
    }
    return goalPose;
  }
}
