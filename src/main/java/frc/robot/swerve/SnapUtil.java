package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
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

  public static double getCoralStationAngle(Pose2d robotPose) {
    if (robotPose.getY() > 4.025) {
      if (FmsSubsystem.isRedAlliance()) {
        // Coral station red, processor side
        return 234.0;
      }

      // Coral station blue, non processor side
      return 306.0;
    } else {
      if (FmsSubsystem.isRedAlliance()) {
        // Coral station red, non processor side
        return 126.0;
      }
      // Coral station blue, processor side
      return 54.0;
    }
  }

  private SnapUtil() {}
}
