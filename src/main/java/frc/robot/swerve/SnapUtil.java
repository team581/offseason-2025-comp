package frc.robot.swerve;

import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import java.util.List;

public class SnapUtil {

  public static double getAmpAngle() {
    return FmsSubsystem.isRedAlliance() ? 90 : (90.0);
  }

  public static double getPodiumAngle() {
    // return whatever the amp angle is
    return FmsSubsystem.isRedAlliance() ? 0 : (180.0);
  }

  public static double getSubwooferAngle() {
    // return whatever the amp angle is
    return FmsSubsystem.isRedAlliance() ? 0 : (180.0);
  }

  private static final List<Double> RED_STAGE_ANGLES = List.of(0.0, 120.0, -120.0);
  private static final List<Double> BLUE_STAGE_ANGLES =
      List.of(0.0 + 180.0, 120.0 - 180.0, -120 + 180.0);

  public static double getClimbingAngle(ImuSubsystem imu) {
    var usedAngles = FmsSubsystem.isRedAlliance() ? RED_STAGE_ANGLES : BLUE_STAGE_ANGLES;
    if (FmsSubsystem.isRedAlliance()) {
      var currentAngle = imu.getRobotHeading();

      var closestAngle = RED_STAGE_ANGLES.get(0);
      var smallestDifference = Double.POSITIVE_INFINITY;
      for (var angle : usedAngles) {
        if (Math.abs(angle - currentAngle) < smallestDifference) {
          closestAngle = angle;
          smallestDifference = Math.abs(angle - currentAngle);
        }
      }

      return closestAngle;
    } else {
      var currentAngle = imu.getRobotHeading();

      var closestAngle = BLUE_STAGE_ANGLES.get(0);
      var smallestDifference = Double.POSITIVE_INFINITY;
      for (var angle : usedAngles) {
        if (Math.abs(angle - currentAngle) < smallestDifference) {
          closestAngle = angle;
          smallestDifference = Math.abs(angle - currentAngle);
        }
      }

      return closestAngle;
    }
  }

  private SnapUtil() {}
}
