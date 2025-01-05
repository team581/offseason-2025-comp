package frc.robot.vision.interpolation;

import java.util.List;

/** A interpolated vision data set for a specific camera. */
public record CameraDataset(
    List<VisionInterpolationData> red, List<VisionInterpolationData> blue) {}
