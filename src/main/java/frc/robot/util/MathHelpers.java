package frc.robot.util;

public class MathHelpers {
  public static double roundTo(double value, double precision) {
    return Math.round(value / precision) * precision;
  }

  private MathHelpers() {}
}
