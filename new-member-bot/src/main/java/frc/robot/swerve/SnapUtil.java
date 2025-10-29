package frc.robot.swerve;

import static java.util.Comparator.comparingDouble;

import com.team581.util.FmsUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.auto_align.ReefSide;
import java.util.Arrays;
import java.util.Collections;

public class SnapUtil {
  public static double getProcessorAngle() {
    return FmsUtil.isRedAlliance() ? 190 : 370;
  }

  public static double getCageAngle(boolean isRedAlliance) {
    return isRedAlliance ? 90 : 270;
  }

  public static double getCageAngle(Pose2d robotPose) {
    return getCageAngle(robotPose.getX() > (17.55 / 2.0));
  }

  public static double getNetScoringAngle(Pose2d robotPose) {
    double robotX = robotPose.getX();
    // entire field length is 17.55m
    double halfFieldLength = 17.55 / 2.0;

    if (robotX < halfFieldLength) {
      return 90.0;
    } else {
      return 270.0;
    }
  }

  public static double getCoralStationAngle(Pose2d robotPose) {
    if (robotPose.getY() > 4.025) {
      if (FmsUtil.isRedAlliance()) {
        // Coral station red, processor side
        return 234.0;
      }

      // Coral station blue, non processor side
      return 306.0;
    } else {
      if (FmsUtil.isRedAlliance()) {
        // Coral station red, non processor side
        return 126.0;
      }
      // Coral station blue, processor side
      return 54.0;
    }
  }

  public static double getNearestReefAngle(Pose2d robotPose) {
    return Collections.min(
            Arrays.asList(ReefSide.values()),
            comparingDouble(
                side ->
                    side.getPose(robotPose)
                        .getTranslation()
                        .getDistance(robotPose.getTranslation())))
        .getPose(robotPose)
        .getRotation()
        .getDegrees();
  }

  private SnapUtil() {}
}
