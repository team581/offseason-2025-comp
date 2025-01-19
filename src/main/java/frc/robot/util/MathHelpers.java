package frc.robot.util;

public class MathHelpers {
  public static double roundTo(double value, double precision) {
    return Math.round(value / precision) * precision;
  }

  public static double sec(double radians) {
    return (1 / Math.cos(radians));
  }

  public static double csc(double radians) {
    return (1 / Math.sin(radians));
  }

  private MathHelpers() {}
}
