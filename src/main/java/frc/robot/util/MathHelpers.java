package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MathHelpers {
  public static double roundTo(double value, double precision) {
    return Math.round(value / precision) * precision;
  }

  public static Translation2d roundTo(Translation2d input, double precision) {
    return new Translation2d(roundTo(input.getX(), precision), roundTo(input.getY(), precision));
  }

  public static double sec(double radians) {
    return (1 / Math.cos(radians));
  }

  public static double csc(double radians) {
    return (1 / Math.sin(radians));
  }

  public static double angleModulus(double angleDegrees) {
    return MathUtil.inputModulus(angleDegrees, -180, 180);
  }

  public static Translation2d chassisSpeedsToTranslation2d(ChassisSpeeds speeds) {
    return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  public static ChassisSpeeds translation2dToChassisSpeeds(Translation2d translation2d) {
    return new ChassisSpeeds(translation2d.getX(), translation2d.getY(), 0.0);
  }

  public static double nonZeroDivide(double a, double b) {
    return b == 0 ? 0 : a / b;
  }

  private MathHelpers() {}
}
