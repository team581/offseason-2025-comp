package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

public record VisionInterpolationData(
    Pose2d measuredPose, Pose2d visionPose, Transform2d transform, String label) {
  public VisionInterpolationData(Pose2d measuredPose, Pose2d visionPose, String label) {
    this(measuredPose, visionPose, new Transform2d(visionPose, measuredPose), label);
  }
}
