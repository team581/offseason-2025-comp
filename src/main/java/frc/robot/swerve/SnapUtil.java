package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.fms.FmsSubsystem;

public class SnapUtil {

  public static double getProcessorAngle() {
    return FmsSubsystem.isRedAlliance() ? 100 : 280;
  }

  public static double getCageAngle() {
    return FmsSubsystem.isRedAlliance() ? 0 : 180;
  }

  public static double getForwardNetDirection(Pose2d robotPose) {
    var robotX = robotPose.getX();
    // entire field length is 17.55m
    double halfFieldLength = 17.55 / 2.0;

    // Robot is on blue side
    if (robotX < halfFieldLength) {
      return 0.0;
    }

    // Robot is on red side
    return 180;
  }

  public static double getBackwardNetDirection(Pose2d robotPose) {
    var robotX = robotPose.getX();
    // entire field length is 17.55m
    double halfFieldLength = 17.55 / 2.0;

    // Robot is on blue side
    if (robotX < halfFieldLength) {
      return 180.0;
    }

    // Robot is on red side
    return 0.0;
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
