package frc.robot.robot_manager.collision_avoidance;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.Optional;

public class CollisionAvoidance {
  private static final double wristLength = 19.0;

  // private static final SuperstructurePosition safePoint2 = new SuperstructurePosition(0.0, 0.0);

  public static Optional<SuperstructurePosition> plan(
      SuperstructurePosition current, SuperstructurePosition goal) {
    var goalPoint = getGoalPoint(current, goal);
    if (goalPoint.isPresent()) {
      DogLog.log("CollisionAvoidance/NextMovePresent", true);

      DogLog.log("CollisionAvoidance/NextMove/elevatorHeight", goalPoint.get().elevatorHeight());
      DogLog.log("CollisionAvoidance/NextMove/wristAngle", goalPoint.get().wristAngle());
    }
    DogLog.log("CollisionAvoidance/NextMovePresent", false);
    return goalPoint;
  }

  static boolean inZone(SuperstructurePosition current, CollisionBoxes collisionBox) {
    Translation2d translation =
        angleHeightToTranslation(current.wristAngle(), current.elevatorHeight());
    Translation2d bottomCorner = collisionBox.box.bottomCorner();
    Translation2d topCorner = collisionBox.box.topCorner();
    if (bottomCorner.getX() <= translation.getX()
        && translation.getX() <= topCorner.getX()
        && bottomCorner.getY() <= translation.getY()
        && translation.getY() <= topCorner.getY()) {
      return true;
    } else {
      return false;
    }
  }

  static CollisionBoxes getZone(SuperstructurePosition current) {
    CollisionBoxes closestBox = CollisionBoxes.BOX_3;
    double closestDistance = Double.MAX_VALUE;
    if (inZone(current, CollisionBoxes.BOX_1)) {
      return CollisionBoxes.BOX_1;
    } else if (inZone(current, CollisionBoxes.BOX_2)) {
      return CollisionBoxes.BOX_2;
    } else if (inZone(current, CollisionBoxes.BOX_3)) {
      return CollisionBoxes.BOX_3;
    } else if (inZone(current, CollisionBoxes.BOX_4)) {
      return CollisionBoxes.BOX_4;
    } else if (inZone(current, CollisionBoxes.BOX_4)) {
      return CollisionBoxes.BOX_4;
    } else if (inZone(current, CollisionBoxes.BOX_5)) {
      return CollisionBoxes.BOX_5;
    } else if (inZone(current, CollisionBoxes.BOX_6)) {
      return CollisionBoxes.BOX_6;
    } else {
      for (int i = 0; i < 7; ) {
        if (closestDistance
            > distancefromTranslations(
                angleHeightToTranslation(current.wristAngle(), current.elevatorHeight()),
                angleHeightToTranslation(
                    numToCollisionBoxes(i).box.safeZone().wristAngle(),
                    numToCollisionBoxes(i).box.safeZone().elevatorHeight()))) {
          closestBox = numToCollisionBoxes(i);
          closestDistance =
              distancefromTranslations(
                  angleHeightToTranslation(current.wristAngle(), current.elevatorHeight()),
                  angleHeightToTranslation(
                      numToCollisionBoxes(i).box.safeZone().wristAngle(),
                      numToCollisionBoxes(i).box.safeZone().elevatorHeight()));
        }
        i++;
      }
      return closestBox;
    }
  }

  private static CollisionBoxes numToCollisionBoxes(int num) {
    if (num == 1) {
      return CollisionBoxes.BOX_1;
    } else if (num == 2) {
      return CollisionBoxes.BOX_2;
    } else if (num == 3) {
      return CollisionBoxes.BOX_3;
    } else if (num == 4) {
      return CollisionBoxes.BOX_4;
    } else if (num == 5) {
      return CollisionBoxes.BOX_5;
    } else {
      return CollisionBoxes.BOX_6;
    }
  }

  private static int collisionBoxToNum(CollisionBoxes zone) {
    return switch (zone) {
      case BOX_1 -> 1;
      case BOX_2 -> 2;
      case BOX_3 -> 3;
      case BOX_4 -> 4;
      case BOX_5 -> 5;
      case BOX_6 -> 6;
    };
  }

  static double distancefromTranslations(
      Translation2d currentTranslation, Translation2d goalTranslation) {
    return Math.sqrt(
        (Math.pow(goalTranslation.getX() - currentTranslation.getX(), 2))
            + (Math.pow(goalTranslation.getY() - currentTranslation.getY(), 2)));
  }

  static Translation2d angleHeightToTranslation(double wristAngle, double elevatorHeight) {
    return new Translation2d(
        Math.cos(Units.degreesToRadians(wristAngle)) * wristLength,
        elevatorHeight + Math.sin(Units.degreesToRadians(wristAngle)) * wristLength);
  }

  private static Optional<SuperstructurePosition> getGoalPoint(
      SuperstructurePosition currentSuperstructurePosition,
      SuperstructurePosition goalSuperstructurePosition) {
    CollisionBoxes currentZone = getZone(currentSuperstructurePosition);
    CollisionBoxes goalZone = getZone(goalSuperstructurePosition);

    // if (MathUtil.isNear(goalZone.box.zoneNum(), currentZone.box.zoneNum(), 1)) {
    //   return Optional.empty();
    // }
    DogLog.log("CollisionAvoidance/currentZone", currentZone);
    DogLog.log("CollisionAvoidance/goalZone", goalZone);
    DogLog.log(
        "CollisionAvoidance/CurrentPosition",
        angleHeightToTranslation(
            currentSuperstructurePosition.wristAngle(),
            currentSuperstructurePosition.elevatorHeight()));

    if (collisionBoxToNum(currentZone) < collisionBoxToNum(goalZone)) {
      return Optional.of(numToCollisionBoxes(collisionBoxToNum(currentZone) + 1).box.safeZone());
    } else if (collisionBoxToNum(currentZone) > collisionBoxToNum(goalZone)) {
      return Optional.of(numToCollisionBoxes(collisionBoxToNum(currentZone) - 1).box.safeZone());
    }

    return Optional.empty();
  }
}
