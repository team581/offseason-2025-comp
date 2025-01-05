package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

public record VisionResult(Pose2d pose, double timestamp) {}
