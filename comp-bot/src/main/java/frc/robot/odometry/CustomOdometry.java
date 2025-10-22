package frc.robot.odometry;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class CustomOdometry {
  public CustomOdometry() {}

  private static Translation2d getModuleDisplacement(
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

    // Finally, calculate the current module translation on the arc and return it as module
    // displacement
    double displacementX = circleCenterX + (radius * Math.cos(currentAngleRadians));
    double displacementY = circleCenterY + (radius * Math.sin(currentAngleRadians));

    // Logging
    // System.out.println("New test -");
    // System.out.println("Circle center x " + circleCenterX);
    // System.out.println("Circle center y " + circleCenterY);
    // System.out.println("Displacement x " + displacementX);
    // System.out.println("Displacement y " + displacementY);

    return new Translation2d(displacementX, displacementY);
  }

  // TODO: add logging for field relative module poses and previous and updated robot pose
  public Pose2d update(
      Pose2d previousRobotPose,
      SwerveModulePosition[] previousWheelPositions,
      SwerveModulePosition[] currentWheelPositions,
      Rotation2d currentGyroAngle) {
    Translation2d[] robotRelativeModuleOffsets = {
      new Translation2d(Inches.of(12), Inches.of(12)),
      new Translation2d(Inches.of(12), Inches.of(-12)),
      new Translation2d(Inches.of(-12), Inches.of(12)),
      new Translation2d(Inches.of(-12), Inches.of(-12))
    };

    // Logging
    // System.out.println("New test -");
    // System.out.println("Previous gyro angle " + previousRobotPose.getRotation());
    // System.out.println("Current gyro angle: " + currentGyroAngle);

    Pose2d[] fieldRelativeModulePosesOfPreviousPose = {
      previousRobotPose.transformBy(
          new Transform2d(robotRelativeModuleOffsets[0], previousRobotPose.getRotation())),
      previousRobotPose.transformBy(
          new Transform2d(robotRelativeModuleOffsets[1], previousRobotPose.getRotation())),
      previousRobotPose.transformBy(
          new Transform2d(robotRelativeModuleOffsets[2], previousRobotPose.getRotation())),
      previousRobotPose.transformBy(
          new Transform2d(robotRelativeModuleOffsets[3], previousRobotPose.getRotation()))
    };

    Translation2d[] moduleDisplacements = {
      getModuleDisplacement(
          previousWheelPositions[0].angle.getRadians(),
          previousWheelPositions[0].distanceMeters,
          currentWheelPositions[0].angle.getRadians(),
          currentWheelPositions[0].distanceMeters),
      getModuleDisplacement(
          previousWheelPositions[1].angle.getRadians(),
          previousWheelPositions[1].distanceMeters,
          currentWheelPositions[1].angle.getRadians(),
          currentWheelPositions[1].distanceMeters),
      getModuleDisplacement(
          previousWheelPositions[2].angle.getRadians(),
          previousWheelPositions[2].distanceMeters,
          currentWheelPositions[2].angle.getRadians(),
          currentWheelPositions[2].distanceMeters),
      getModuleDisplacement(
          previousWheelPositions[3].angle.getRadians(),
          previousWheelPositions[3].distanceMeters,
          currentWheelPositions[3].angle.getRadians(),
          currentWheelPositions[3].distanceMeters)
    };

    Translation2d[] fieldRelativeModuleDisplacements = {
      fieldRelativeModulePosesOfPreviousPose[0]
          .transformBy(new Transform2d(moduleDisplacements[0], new Rotation2d(0.0)))
          .getTranslation(),
      fieldRelativeModulePosesOfPreviousPose[1]
          .transformBy(new Transform2d(moduleDisplacements[1], new Rotation2d(0.0)))
          .getTranslation(),
      fieldRelativeModulePosesOfPreviousPose[2]
          .transformBy(new Transform2d(moduleDisplacements[2], new Rotation2d(0.0)))
          .getTranslation(),
      fieldRelativeModulePosesOfPreviousPose[3]
          .transformBy(new Transform2d(moduleDisplacements[3], new Rotation2d(0.0)))
          .getTranslation()
    };

    // Divide sum of field relative module displacements by 4 because there are 4 modules
    Translation2d sumOfFieldRelativeModuleDisplacements =
        fieldRelativeModuleDisplacements[0]
            .plus(fieldRelativeModuleDisplacements[1])
            .plus(fieldRelativeModuleDisplacements[2])
            .plus(fieldRelativeModuleDisplacements[3]);
    double displacementX = sumOfFieldRelativeModuleDisplacements.getX() / 4.0;
    double displacementY = sumOfFieldRelativeModuleDisplacements.getY() / 4.0;

    return new Pose2d(displacementX, displacementY, currentGyroAngle);
  }
}
