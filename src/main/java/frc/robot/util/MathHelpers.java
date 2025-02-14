package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  public static Rotation2d angleMin(Rotation2d a, Rotation2d b) {
    return Rotation2d.fromDegrees(Math.min(a.getDegrees(), b.getDegrees()));
  }

  public static Rotation2d angleMax(Rotation2d a, Rotation2d b) {
    return Rotation2d.fromDegrees(Math.max(a.getDegrees(), b.getDegrees()));
  }

  public static Rotation2d angleAbs(Rotation2d a) {
    return Rotation2d.fromDegrees(Math.abs(a.getDegrees()));
  }

  public static Pose2d poseLookahead(Pose2d current, ChassisSpeeds velocity, double lookahead) {
    var x = current.getX() + velocity.vxMetersPerSecond * lookahead;
    var y = current.getY() + velocity.vyMetersPerSecond * lookahead;
    var theta =
        current
            .getRotation()
            .plus(Rotation2d.fromRadians(velocity.omegaRadiansPerSecond * lookahead));

    return new Pose2d(x, y, theta);
  }

  public static double signedExp(double value, double exponent) {
    return Math.copySign(Math.pow(Math.abs(value), exponent), value);
  }

  public static double signedSqrt(double value) {
    return Math.copySign(Math.sqrt(Math.abs(value)), value);
  }

  private MathHelpers() {}
}
