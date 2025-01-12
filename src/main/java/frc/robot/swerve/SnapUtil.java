package frc.robot.swerve;

import frc.robot.fms.FmsSubsystem;

public class SnapUtil {

  public static double getProcessorAngle() {
    return FmsSubsystem.isRedAlliance() ? 270 : 90.0;
  }

  private SnapUtil() {}
}
