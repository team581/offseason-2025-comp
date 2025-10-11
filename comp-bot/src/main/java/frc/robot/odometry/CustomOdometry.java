package frc.robot.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

// TODO: Start basic, then add more complexity over time
public class CustomOdometry {
  private final Rotation2d gyroAngle;
  private final Pose2d pose;

  public CustomOdometry(Rotation2d initialGyroAngle, Pose2d initialPoseMeters) {
    this.gyroAngle = initialGyroAngle;
    this.pose = initialPoseMeters;
  }

  // TODO: account for arc direction
  private Translation2d getModuleDisplacement(
      double previousAngle, double previousDistance, double currentAngle, double currentDistance) {
    // First, calculate difference between previous and current angles and distances
    double angleDifference = Math.abs(currentAngle - previousAngle);
    double arcLength = currentDistance - previousDistance;

    // Next, calculate radius
    double radius = (Math.abs(arcLength) / angleDifference) * (180 / Math.PI);

    // Then, calculate the center point of the circle that the arc is a part of, using the previous
    // angle. The previous module translation is (0, 0) because we don't care where it starts, only
    // the displacement
    double circleCenterX = 0 - (radius * Math.cos(previousAngle));
    double circleCenterY = 0 - (radius * Math.sin(previousAngle));

    // Finally, calculate the current module translation
    double displacementX = circleCenterX + (radius * Math.cos(currentAngle));
    double displacementY = circleCenterY + (radius * Math.cos(currentAngle));

    return new Translation2d(displacementX, displacementY);
  }

  public Pose2d update(
      SwerveModulePosition[] previousWheelPositions, SwerveModulePosition[] currentWheelPositions) {
    // Take averages of module displacements to get robot displacement
    Translation2d robotDisplacement = new Translation2d();

    return new Pose2d(new Translation2d(), gyroAngle);
  }
}
