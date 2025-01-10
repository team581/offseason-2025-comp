package frc.robot.vision.results;

import edu.wpi.first.math.geometry.Pose2d;

public record TagResult(Pose2d pose, double timestamp) {}
