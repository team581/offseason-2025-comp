package frc.robot.odometry;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class CustomOdometry extends SwerveDriveOdometry {
  private SwerveModulePosition[] previousWheelPositions = new SwerveModulePosition[] {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };
  private Pose2d previousRobotPose = new Pose2d();
  private Pose2d[] fieldRelativeModulePosesOfPreviousPoseLogged = new Pose2d[] {
    new Pose2d(),
    new Pose2d(),
    new Pose2d(),
    new Pose2d()
  };

  public CustomOdometry(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions) {
    super(kinematics, gyroAngle, modulePositions);
  }

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
      return new Translation2d(arcLength, new Rotation2d(currentAngleRadians));
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

  // Function to set the previous robot pose, as it can't be passed in to Update() as a parameter
  public void setPreviousRobotPose(Pose2d newPreviousRobotPose) {
    previousRobotPose = newPreviousRobotPose;
  }

  // TODO: add logging for field relative module poses and previous and updated robot pose
  @Override
  public Pose2d update(Rotation2d currentGyroAngle, SwerveModulePosition[] currentWheelPositions) {
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

    fieldRelativeModulePosesOfPreviousPoseLogged = fieldRelativeModulePosesOfPreviousPose;

    // Divide sum of field relative module displacements by 4 because there are 4 modules
    Translation2d sumOfFieldRelativeModuleDisplacements =
        fieldRelativeModuleDisplacements[0]
            .plus(fieldRelativeModuleDisplacements[1])
            .plus(fieldRelativeModuleDisplacements[2])
            .plus(fieldRelativeModuleDisplacements[3]);
    double displacementX = sumOfFieldRelativeModuleDisplacements.getX() / 4.0;
    double displacementY = sumOfFieldRelativeModuleDisplacements.getY() / 4.0;

    // After calculations, before the next loop update the previous wheel positions to the current
    // ones
    updatePreviousWheelPositions(currentWheelPositions);

    return new Pose2d(displacementX, displacementY, currentGyroAngle);
  }

  private void updatePreviousWheelPositions(SwerveModulePosition[] currentWheelPositions) {
    for (int i = 0; i < 4; i++) {
      previousWheelPositions[i] = currentWheelPositions[i];
    }
  }

  public Pose2d[] getFieldRelativeModulePosesOfPreviousPose() {
    return fieldRelativeModulePosesOfPreviousPoseLogged;
  }
}
