package frc.robot.robot_manager.collision_avoidance;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class CollisionAvoidance {
  private static final double wristLength = 22.0;
  private static final SuperstructurePosition safePoint1 = new SuperstructurePosition(0.0, 0.0);
  private static final SuperstructurePosition safePoint2 = new SuperstructurePosition(90.0, 0.0);

  // private static final SuperstructurePosition safePoint2 = new SuperstructurePosition(0.0, 0.0);
  private static final SuperstructurePosition[] corners =
      new SuperstructurePosition[] {
        new SuperstructurePosition(7.4436508139, 135.0),
        new SuperstructurePosition(65.0, 85.0) // h=22-\sin(a)*22 + 1(for clearence) -> a = 135
      };

  public static Optional<SuperstructurePosition> plan(
      SuperstructurePosition current, SuperstructurePosition goal) {
    var possibleGoalPoints = List.of(goal, safePoint1, safePoint2);

    // possibleGoalPoints.set(2, safePoint2);
    //  getGoalPoint(possibleGoalPoints, angleHeightToPose(wristAngle, elevatorHeight))

    return getGoalPoint(possibleGoalPoints, current, goal);
  }

  private CollisionAvoidance() {}

  static boolean collides(Translation2d currentPose, Translation2d goalPose) {

    double x2 = angleHeightToPose(corners[1].wristAngle(), corners[1].elevatorHeight()).getX();
    double y2 = angleHeightToPose(corners[1].wristAngle(), corners[1].elevatorHeight()).getY();
    double y1 = angleHeightToPose(corners[0].wristAngle(), corners[0].elevatorHeight()).getY();
    double x1 = angleHeightToPose(corners[0].wristAngle(), corners[0].elevatorHeight()).getX();

    double currentPoseX = currentPose.getX();
    double currentPoseY = currentPose.getY();

    double goalPoseX = goalPose.getX();
    double goalPoseY = goalPose.getY();
    // If the points are (x1, y1) and (x2, y2), then the two-point form reads:

    // y - y1 = (y2 - y1)/(x2 - x1) × (x - x1).
    Translation2d xInterceptionPoint =
        new Translation2d(
            x2,
            (goalPoseY - currentPoseY) / (goalPoseX - currentPoseX) * (x2 - currentPoseX)
                + currentPoseY);
    Translation2d yInterceptionPoint =
        new Translation2d(
            ((goalPoseX - currentPoseX) * (y1 - currentPoseY)
                    + (goalPoseY - currentPoseY) * currentPoseX)
                / (goalPoseY - currentPoseY),
            y1);
    Translation2d y2InterceptionPoint =
        new Translation2d(
            ((goalPoseX - currentPoseX) * (y2 - currentPoseY)
                    + (goalPoseY - currentPoseY) * currentPoseX)
                / (goalPoseY - currentPoseY),
            y2);

    if (y1 < xInterceptionPoint.getY() && xInterceptionPoint.getY() < y2
        || x1 < yInterceptionPoint.getX() && yInterceptionPoint.getX() < x2
        || x1 < y2InterceptionPoint.getX() && y2InterceptionPoint.getX() < x2) {
      return true;
    }

    return false;
  }

  static Translation2d angleHeightToPose(double wristAngle, double elevatorHeight) {
    return new Translation2d(
        Math.cos(Units.degreesToRadians(wristAngle)) * wristLength,
        elevatorHeight + Math.sin(Units.degreesToRadians(wristAngle)) * wristLength);
  }

  // static SuperstructurePosition poseToSuperstructurePosition(Translation2d pose) {
  //   if(180>wristAngle&&wristAngle>0){
  //     return new SuperstructurePosition(
  //       pose.getY()-Math.sqrt(Math.pow(wristLength, 2)-Math.pow(pose.getX(), 2)),
  //       Units.radiansToDegrees(Math.acos(pose.getX()/wristLength))
  //   );
  //   } else{return new SuperstructurePosition(
  //       pose.getY()+Math.sqrt(Math.pow(wristLength, 2)-Math.pow(pose.getX(), 2)),
  //       //WE SHOULD ADD INSTEAD OF SUBTRACT IN FRONT OF MATH.SQRT IF we Are BELoW 90 degree but
  // idk if we are ever below
  //       -1*Units.radiansToDegrees(Math.acos(pose.getX()/wristLength))
  //   );}

  // }

  static double distancefromPoses(Translation2d currentPose, Translation2d goalPose) {
    return Math.sqrt(
        (Math.pow(goalPose.getX() - currentPose.getX(), 2))
            + (Math.pow(goalPose.getY() - currentPose.getY(), 2)));
  }

  private static Optional<SuperstructurePosition> getGoalPoint(
      List<SuperstructurePosition> possibleGoalPoints,
      SuperstructurePosition currentSuperstructurePosition,
      SuperstructurePosition goalSuperstructurePosition) {
    ArrayList<SuperstructurePosition> availablePoints = new ArrayList<SuperstructurePosition>();
    if (!collides(
        angleHeightToPose(
            currentSuperstructurePosition.wristAngle(),
            currentSuperstructurePosition.elevatorHeight()),
        angleHeightToPose(
            goalSuperstructurePosition.wristAngle(),
            goalSuperstructurePosition.elevatorHeight()))) {
      return Optional.empty();
    }
    for (int i = 0; i < possibleGoalPoints.size(); ) {
      if (!collides(
          angleHeightToPose(
              currentSuperstructurePosition.wristAngle(),
              currentSuperstructurePosition.elevatorHeight()),
          angleHeightToPose(
              possibleGoalPoints.get(i).wristAngle(),
              possibleGoalPoints.get(i).elevatorHeight()))) {
        availablePoints.add(possibleGoalPoints.get(i));
      }
      i++;
    }
    double closestDistance = Double.MAX_VALUE;
    SuperstructurePosition closestPossiblePose = new SuperstructurePosition(999, 999);
    for (int w = 0; w < availablePoints.size(); ) {

      if (distancefromPoses(
              angleHeightToPose(
                  goalSuperstructurePosition.wristAngle(),
                  goalSuperstructurePosition.elevatorHeight()),
              angleHeightToPose(
                  availablePoints.get(w).wristAngle(), availablePoints.get(w).elevatorHeight()))
          < closestDistance) {
        closestDistance =
            distancefromPoses(
                angleHeightToPose(
                    goalSuperstructurePosition.wristAngle(),
                    goalSuperstructurePosition.elevatorHeight()),
                angleHeightToPose(
                    availablePoints.get(w).wristAngle(), availablePoints.get(w).elevatorHeight()));
        closestPossiblePose = availablePoints.get(w);
      }
      w++;
    }
    return Optional.of(closestPossiblePose);
  }
}
