package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

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

  private MathHelpers() {}
}
