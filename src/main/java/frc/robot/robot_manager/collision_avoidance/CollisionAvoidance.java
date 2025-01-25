package frc.robot.robot_manager.collision_avoidance;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class CollisionAvoidance {
  private static final double wristLength = 19.0;
  private static final SuperstructurePosition safePoint1 = new SuperstructurePosition(0.0, 0.0);
  private static final SuperstructurePosition safePoint2 = new SuperstructurePosition(90.0, 0.0);

  // private static final SuperstructurePosition safePoint2 = new SuperstructurePosition(0.0, 0.0);
  private static final SuperstructurePosition[] corners =
      new SuperstructurePosition[] {
        new SuperstructurePosition(5.7, 135.0),
        new SuperstructurePosition(57.0, 15.0) // h=22-\sin(a)*22 + 1(for clearence) -> a = 135
      };

  public static Optional<SuperstructurePosition> plan(
      SuperstructurePosition current, SuperstructurePosition goal) {
    var possibleGoalPoints = List.of(goal, safePoint1, safePoint2);

    DogLog.log("CollisionAvoidance/isInBadZone", isInBadZone(current));
    return getGoalPoint(possibleGoalPoints, current, goal);
  }

  private CollisionAvoidance() {}

  private static boolean isInBadZone(SuperstructurePosition current) {
    Translation2d translation =
        angleHeightToTranslation(current.wristAngle(), current.elevatorHeight());
    Translation2d bottomCorner =
        angleHeightToTranslation(corners[0].wristAngle(), corners[0].elevatorHeight());
    Translation2d topCorner =
        angleHeightToTranslation(corners[1].wristAngle(), corners[1].elevatorHeight());
    if (bottomCorner.getX() < translation.getX()
        && translation.getX() < topCorner.getX()
        && bottomCorner.getY() < translation.getY()
        && translation.getY() < topCorner.getY()) {
      return true;
    }
    return false;
  }

  static boolean collides(Translation2d currentTranslation, Translation2d goalTranslation) {

    double x2 =
        angleHeightToTranslation(corners[1].wristAngle(), corners[1].elevatorHeight()).getX();
    double y2 =
        angleHeightToTranslation(corners[1].wristAngle(), corners[1].elevatorHeight()).getY();
    double y1 =
        angleHeightToTranslation(corners[0].wristAngle(), corners[0].elevatorHeight()).getY();
    double x1 =
        angleHeightToTranslation(corners[0].wristAngle(), corners[0].elevatorHeight()).getX();

    double currentTranslationX = currentTranslation.getX();
    double currentTranslationY = currentTranslation.getY();

    double goalTranslationX = goalTranslation.getX();
    double goalTranslationY = goalTranslation.getY();
    // If the points are (x1, y1) and (x2, y2), then the two-point form reads:

    // y - y1 = (y2 - y1)/(x2 - x1) × (x - x1).
    Translation2d xInterceptionPoint =
        new Translation2d(
            x2,
            (goalTranslationY - currentTranslationY)
                    / (goalTranslationX - currentTranslationX)
                    * (x2 - currentTranslationX)
                + currentTranslationY);
    Translation2d yInterceptionPoint =
        new Translation2d(
            ((goalTranslationX - currentTranslationX) * (y1 - currentTranslationY)
                    + (goalTranslationY - currentTranslationY) * currentTranslationX)
                / (goalTranslationY - currentTranslationY),
            y1);
    Translation2d y2InterceptionPoint =
        new Translation2d(
            ((goalTranslationX - currentTranslationX) * (y2 - currentTranslationY)
                    + (goalTranslationY - currentTranslationY) * currentTranslationX)
                / (goalTranslationY - currentTranslationY),
            y2);

    if (y1 < xInterceptionPoint.getY() && xInterceptionPoint.getY() < y2
        || x1 < yInterceptionPoint.getX() && yInterceptionPoint.getX() < x2
        || x1 < y2InterceptionPoint.getX() && y2InterceptionPoint.getX() < x2) {
      return true;
    }

    return false;
  }

  static Translation2d angleHeightToTranslation(double wristAngle, double elevatorHeight) {
    return new Translation2d(
        Math.cos(Units.degreesToRadians(wristAngle)) * wristLength,
        elevatorHeight + Math.sin(Units.degreesToRadians(wristAngle)) * wristLength);
  }

  static double distancefromTranslations(
      Translation2d currentTranslation, Translation2d goalTranslation) {
    return Math.sqrt(
        (Math.pow(goalTranslation.getX() - currentTranslation.getX(), 2))
            + (Math.pow(goalTranslation.getY() - currentTranslation.getY(), 2)));
  }

  private static Optional<SuperstructurePosition> getGoalPoint(
      List<SuperstructurePosition> possibleGoalPoints,
      SuperstructurePosition currentSuperstructurePosition,
      SuperstructurePosition goalSuperstructurePosition) {
    ArrayList<SuperstructurePosition> availablePoints = new ArrayList<SuperstructurePosition>();
    if (!collides(
        angleHeightToTranslation(
            currentSuperstructurePosition.wristAngle(),
            currentSuperstructurePosition.elevatorHeight()),
        angleHeightToTranslation(
            goalSuperstructurePosition.wristAngle(),
            goalSuperstructurePosition.elevatorHeight()))) {
      return Optional.empty();
    }
    for (int i = 0; i < possibleGoalPoints.size(); ) {
      if (!collides(
          angleHeightToTranslation(
              currentSuperstructurePosition.wristAngle(),
              currentSuperstructurePosition.elevatorHeight()),
          angleHeightToTranslation(
              possibleGoalPoints.get(i).wristAngle(),
              possibleGoalPoints.get(i).elevatorHeight()))) {
        availablePoints.add(possibleGoalPoints.get(i));
      }
      i++;
    }
    double closestDistance = Double.MAX_VALUE;
    SuperstructurePosition closestPossibleTranslation = new SuperstructurePosition(999, 999);
    for (int w = 0; w < availablePoints.size(); ) {

      if (distancefromTranslations(
              angleHeightToTranslation(
                  goalSuperstructurePosition.wristAngle(),
                  goalSuperstructurePosition.elevatorHeight()),
              angleHeightToTranslation(
                  availablePoints.get(w).wristAngle(), availablePoints.get(w).elevatorHeight()))
          < closestDistance) {
        closestDistance =
            distancefromTranslations(
                angleHeightToTranslation(
                    goalSuperstructurePosition.wristAngle(),
                    goalSuperstructurePosition.elevatorHeight()),
                angleHeightToTranslation(
                    availablePoints.get(w).wristAngle(), availablePoints.get(w).elevatorHeight()));
        closestPossibleTranslation = availablePoints.get(w);
      }
      w++;
    }
    return Optional.of(closestPossibleTranslation);
  }
}
