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

  private Translation2d getModuleDisplacement(
      double previousAngleRadians,
      double previousDistanceMeters,
      double currentAngleRadians,
      double currentDistanceMeters) {
    // First, calculate difference between previous and current angles and distances
    double angleDifferenceRadians = Math.abs(currentAngleRadians - previousAngleRadians);
    double arcLength = currentDistanceMeters - previousDistanceMeters;

    // If angle difference is 0 then we can just use a straight line instead of an arc
    if (angleDifferenceRadians == 0) {
      double displacementX = arcLength * Math.cos(currentAngleRadians);
      double displacementY = arcLength * Math.sin(currentAngleRadians);

      return new Translation2d(displacementX, displacementY);
    }

    // Next, calculate radius
    double radius = (Math.abs(arcLength) / angleDifferenceRadians);

    // Then, calculate the center point of the circle that the arc is a part of, using the previous
    // angle. The previous module translation is (0, 0) because we don't care where it starts, only
    // the displacement
    double circleCenterX = 0 - (radius * Math.cos(previousAngleRadians));
    double circleCenterY = 0 - (radius * Math.sin(previousAngleRadians));

    // Finally, calculate the current module translation. If arc length is negative, invert
    // displacement
    double displacementX = circleCenterX + (radius * Math.cos(currentAngleRadians));
    double displacementY = circleCenterY + (radius * Math.sin(currentAngleRadians));

    if (arcLength < 0) {
      displacementX *= -1;
      displacementY *= -1;
    }

    return new Translation2d(displacementX, displacementY);
  }

  public Pose2d update(
      SwerveModulePosition[] previousWheelPositions, SwerveModulePosition[] currentWheelPositions) {
    Translation2d sumOfModuleDisplacements =
        getModuleDisplacement(
                previousWheelPositions[0].angle.getRadians(),
                previousWheelPositions[0].distanceMeters,
                currentWheelPositions[0].angle.getRadians(),
                currentWheelPositions[0].distanceMeters)
            .plus(
                getModuleDisplacement(
                    previousWheelPositions[1].angle.getRadians(),
                    previousWheelPositions[1].distanceMeters,
                    currentWheelPositions[1].angle.getRadians(),
                    currentWheelPositions[1].distanceMeters))
            .plus(
                getModuleDisplacement(
                    previousWheelPositions[2].angle.getRadians(),
                    previousWheelPositions[2].distanceMeters,
                    currentWheelPositions[2].angle.getRadians(),
                    currentWheelPositions[2].distanceMeters))
            .plus(
                getModuleDisplacement(
                    previousWheelPositions[3].angle.getRadians(),
                    previousWheelPositions[3].distanceMeters,
                    currentWheelPositions[3].angle.getRadians(),
                    currentWheelPositions[3].distanceMeters));

    // Divide sum of module displacements by the amount of swerve modules, 4
    Translation2d robotDisplacement =
        new Translation2d(
            sumOfModuleDisplacements.getX() / 4.0, sumOfModuleDisplacements.getY() / 4.0);

    return new Pose2d(robotDisplacement, gyroAngle);
  }
}
