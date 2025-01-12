package frc.robot.swerve;

import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;

public class SnapUtil {

  public static double getProcessorAngle() {
    return FmsSubsystem.isRedAlliance() ? 270 : 90.0;
  }
  
  private SnapUtil() {}
}
