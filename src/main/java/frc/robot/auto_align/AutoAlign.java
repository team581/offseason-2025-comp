package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.fms.FmsSubsystem;
import java.util.List;

public class AutoAlign {
  private static final List<ReefSide> ALL_REEF_SIDES = List.of(ReefSide.values());

  public static Pose2d getClosestReefSide(Pose2d robotPose) {
    var reefSide =
        ALL_REEF_SIDES.stream()
            .min(
                (a, b) ->
                    Double.compare(
                        robotPose
                            .getTranslation()
                            .getDistance(
                                (FmsSubsystem.isRedAlliance() ? a.redPose : a.bluePose)
                                    .getTranslation()),
                        robotPose
                            .getTranslation()
                            .getDistance(
                                (FmsSubsystem.isRedAlliance() ? b.redPose : b.bluePose)
                                    .getTranslation())))
            .get();

    if (FmsSubsystem.isRedAlliance()) {
      return reefSide.redPose;
    }

    return reefSide.bluePose;
  }

  public static boolean shouldNetScoreForwards(Pose2d robotPose) {
    double robotX = robotPose.getX();
    double theta = robotPose.getRotation().getDegrees();

    // entire field length is 17.55m
    double halfFieldLength = 17.55 / 2.0;

    // Robot is on blue side
    if (robotX < halfFieldLength) {
      return theta < 90 || theta > 270;
    }

    // Robot is on red side
    return theta > 90 && theta < 270;
  }
}
