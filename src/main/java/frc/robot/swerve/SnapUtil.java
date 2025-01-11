package frc.robot.swerve;

import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;

public class SnapUtil {

  public static double getProcessorAngle() {
    return FmsSubsystem.isRedAlliance() ? 270 : 90.0;
  }

  public static double getNetAngle(ImuSubsystem imu) {
    if (Math.abs(Math.abs(imu.getRobotHeading()) - 180.0)
        <= Math.abs(Math.abs(imu.getRobotHeading()) - 0)) {
      return 180.0;
    }

    return 0.0;
  }

  private SnapUtil() {}
}
