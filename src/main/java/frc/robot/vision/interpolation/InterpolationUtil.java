package frc.robot.vision.interpolation;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

public class InterpolationUtil {
  private static final boolean USE_LINEAR_WEIGHTS = false;

  public static Translation2d interpolateTranslation(
      List<VisionInterpolationData> dataPoints, Translation2d visionInput) {
    var unnormalizedWeightsSum =
        dataPoints.stream()
            .mapToDouble(
                dataPoint ->
                    calculateUnnormalizedWeight(dataPoint.visionPose().getDistance(visionInput)))
            .sum();

    double weightedX = 0;
    double weightedY = 0;

    for (var dataPoint : dataPoints) {
      var distancePoint = dataPoint.visionPose().getDistance(visionInput);
      var weight = calculateUnnormalizedWeight(distancePoint) / unnormalizedWeightsSum;
      var result = dataPoint.measuredPose().minus(dataPoint.visionPose()).times(weight);
      weightedX += result.getX();
      weightedY += result.getY();
    }

    return new Translation2d(visionInput.getX() + weightedX, visionInput.getY() + weightedY);
  }

  private static double calculateUnnormalizedWeight(double distance) {
    if (USE_LINEAR_WEIGHTS) {
      return (1.0 + distance) / distance;
    }

    return 1.0 / Math.pow(distance, 2);
  }

  private InterpolationUtil() {}
}
