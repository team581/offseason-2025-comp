package frc.robot.swerve;

import frc.robot.fms.FmsSubsystem;

public class SnapUtil {

  public static double getProcessorAngle() {
    return FmsSubsystem.isRedAlliance() ? 270 : 90.0;
  }

  public static double getForwardNetDirection() {
    return FmsSubsystem.isRedAlliance() ? 0 : 180;
  }

  public static double getBackwardNetDirection() {
    return FmsSubsystem.isRedAlliance() ? 180 : 0;
  }

  private SnapUtil() {}
}
