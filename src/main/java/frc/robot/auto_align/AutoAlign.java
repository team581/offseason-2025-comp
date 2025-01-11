package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.fms.FmsSubsystem;
import java.util.List;

public class AutoAlign {
  private static final List<ReefSide> ALL_REEF_SIDES = List.of(ReefSide.values());

  public static ReefSide getClosestReefSide(Pose2d robotPose) {
    return ALL_REEF_SIDES.stream()
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
  }
}
