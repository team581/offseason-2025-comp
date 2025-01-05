package frc.robot.vision.interpolation;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.fms.FmsSubsystem;

public class InterpolatedVision {
  public static Pose2d interpolatePose(Pose2d visionInput, CameraDataset dataset) {
    var usedDataPoints = FmsSubsystem.isRedAlliance() ? dataset.red() : dataset.blue();

    return new Pose2d(
        InterpolationUtil.interpolateTranslation(usedDataPoints, visionInput.getTranslation()),
        visionInput.getRotation());
  }

  private InterpolatedVision() {}
}
