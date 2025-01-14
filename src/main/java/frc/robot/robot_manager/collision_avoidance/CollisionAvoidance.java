package frc.robot.robot_manager.collision_avoidance;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.ArrayList;
import java.util.Optional;

public class CollisionAvoidance {
  private static final double wristLength = 22.0;
  private static final SuperstructurePosition safePoint1 = new SuperstructurePosition(0.0, 0.0);
  private static final SuperstructurePosition safePoint2 = new SuperstructurePosition(90.0, 0.0);

 // private static final SuperstructurePosition safePoint2 = new SuperstructurePosition(0.0, 0.0);
  private static final SuperstructurePosition[] corners =
      new SuperstructurePosition[] {
        new SuperstructurePosition(2.0, 135.0), new SuperstructurePosition(65.0, 85.0)
      }; // TOP RIGHT CORNER IS 0 and TOP RIGHT CORNER IS 1
  static ArrayList<SuperstructurePosition> possibleGoalPoints = new ArrayList<SuperstructurePosition>();


    public static Optional<SuperstructurePosition> plan(
        SuperstructurePosition current, SuperstructurePosition goal) {
      return plan(
          current.elevatorHeight(), current.wristAngle(), goal.elevatorHeight(), goal.wristAngle());
    }

    public static Optional<SuperstructurePosition> plan(
        double elevatorHeight, double wristAngle, double elevatorGoal, double wristGoal) {
          possibleGoalPoints = new ArrayList<SuperstructurePosition>();
      possibleGoalPoints.add( new SuperstructurePosition(elevatorGoal, wristGoal));
      possibleGoalPoints.add( safePoint1);
      possibleGoalPoints.add(safePoint2);
     // possibleGoalPoints.set(2, safePoint2);
      //  getGoalPoint(possibleGoalPoints, angleHeightToPose(wristAngle, elevatorHeight))

      return Optional.of(
          poseToSuperstructurePosition(
              getGoalPoint(possibleGoalPoints, angleHeightToPose(wristAngle, elevatorHeight),angleHeightToPose(wristGoal, elevatorGoal)),wristAngle));
    }

    private CollisionAvoidance() {}

    static boolean collides(Translation2d currentPose, Translation2d goalPose) {

      double x1 = angleHeightToPose(corners[0].wristAngle(), corners[0].elevatorHeight()).getX();
      double y1 = angleHeightToPose(corners[0].wristAngle(), corners[0].elevatorHeight()).getY();
      double y2 = angleHeightToPose(corners[1].wristAngle(), corners[1].elevatorHeight()).getY();
      double x2 = angleHeightToPose(corners[1].wristAngle(), corners[1].elevatorHeight()).getX();

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

    static Translation2d angleHeightToPose(double wristAngle, double elevatorHeight) {
      return new Translation2d(
          Math.cos(Units.degreesToRadians(wristAngle)) * wristLength,
          elevatorHeight + Math.sin(Units.degreesToRadians(wristAngle)) * wristLength);
    }

    static SuperstructurePosition poseToSuperstructurePosition(Translation2d pose, double wristAngle) {
      if(wristAngle>0){
        return new SuperstructurePosition(
          pose.getY()+Math.sqrt(Math.pow(wristLength, 2)-Math.pow(pose.getX(), 2)),
          Units.radiansToDegrees(Math.acos(pose.getX()/wristLength))
      );
      } else{return new SuperstructurePosition(
          pose.getY()-Math.sqrt(Math.pow(wristLength, 2)-Math.pow(pose.getX(), 2)),
          //WE SHOULD ADD INSTEAD OF SUBTRACT IN FRONT OF MATH.SQRT IF we Are BELoW 90 degree but idk if we are ever below
          Units.radiansToDegrees(Math.acos(pose.getX()/wristLength))
      );}

    }

    static double distancefromPoses(Translation2d currentPose, Translation2d goalPose) {
      return Math.sqrt(
          (Math.pow(goalPose.getX() - currentPose.getX(),2)) + (Math.pow(goalPose.getY() - currentPose.getY(),2)));
    }

    private static Translation2d getGoalPoint(ArrayList<SuperstructurePosition> possibleGoalPoints, Translation2d currentPose, Translation2d goalPose) {
      ArrayList<SuperstructurePosition> availablePoints = new ArrayList<SuperstructurePosition>();
      if(!collides(currentPose, goalPose)){
        return goalPose;
      }
      for (int i = 0; i < possibleGoalPoints.size(); ) {
        if (!collides(currentPose, angleHeightToPose(possibleGoalPoints.get(i).wristAngle(), possibleGoalPoints.get(i).elevatorHeight()))) {
          availablePoints.add(possibleGoalPoints.get(i));
        }
        i++;
      }
      double closestDistance = Double.MAX_VALUE;
      Translation2d closestPossiblePose = new Translation2d();
      for (int w = 0; w < availablePoints.size(); ) {

        if (distancefromPoses(
                goalPose,
                angleHeightToPose(
                    availablePoints.get(w).wristAngle(), availablePoints.get(w).elevatorHeight()))
            < closestDistance) {

          closestPossiblePose =
            angleHeightToPose(
                availablePoints.get(w).wristAngle(), availablePoints.get(w).elevatorHeight());
      }
      w++;
    }
    return closestPossiblePose;
  }
}
