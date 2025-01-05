package frc.robot.vision.interpolation;

import edu.wpi.first.math.geometry.Translation2d;

public record VisionInterpolationData(
    Translation2d measuredPose, Translation2d visionPose, String label) {}
