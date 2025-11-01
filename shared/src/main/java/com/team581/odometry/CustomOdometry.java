package com.team581.odometry;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class CustomOdometry extends SwerveDriveOdometry {
  private final int numberOfModules;
  private final Translation2d[] robotRelativeModuleOffsets;

  private Pose2d previousRobotPose = new Pose2d();
  private final SwerveModulePosition[] previousWheelPositions;

  public CustomOdometry(
      int numberOfModules,
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions) {
    super(kinematics, gyroAngle, modulePositions);
    this.numberOfModules = numberOfModules;
    robotRelativeModuleOffsets = new Translation2d[numberOfModules];
    previousWheelPositions = new SwerveModulePosition[numberOfModules];

    for (int i = 0; i < numberOfModules; i++) {
      robotRelativeModuleOffsets[i] =
          new Translation2d(kinematics.getModules()[i].getX(), kinematics.getModules()[i].getY());
      previousWheelPositions[i] = new SwerveModulePosition();
    }
  }

  private static Translation2d getModuleDisplacement(
      double previousAngleRadians,
      double previousDistanceMeters,
      double currentAngleRadians,
      double currentDistanceMeters) {
    // First, calculate difference between previous and current angles and distances
    double angleDifferenceRadians = currentAngleRadians - previousAngleRadians;
    double arcLength = currentDistanceMeters - previousDistanceMeters;

    // *If angle difference is 0 then we can just use a straight line instead of an arc
    if (angleDifferenceRadians == 0) {
      return new Translation2d(arcLength, new Rotation2d(currentAngleRadians));
    }

    // Next, calculate radius. Positive = left turn, negative = right turn
    double radius = (arcLength / angleDifferenceRadians);
    DogLog.log("Odometry/GetModuleDisplacement/Radius", radius);

    // Then, calculate the center point of the circle that the arc is a part of, using the previous
    // angle. The previous module translation is (0, 0) because we don't care where it starts, only
    // the displacement. It is also always perpendicular to the preivous angle, with
    // positive/negative radius indicating which side of the module that the circle center will be
    // on
    double circleCenterX = -radius * Math.sin(previousAngleRadians);
    double circleCenterY = radius * Math.cos(previousAngleRadians);

    // Finally, calculate the current module translation on the arc and return it as module
    // displacement
    double displacementX = circleCenterX + radius * Math.sin(currentAngleRadians);
    double displacementY = circleCenterY - radius * Math.cos(currentAngleRadians);

    return new Translation2d(displacementX, displacementY);
  }

  @Override
  public Pose2d update(Rotation2d currentGyroAngle, SwerveModulePosition[] currentWheelPositions) {
    // First, get the field relative module poses of the previous robot pose, and apply robot
    // relative module offsets
    Pose2d[] fieldRelativeModulePosesOfPreviousPose = new Pose2d[numberOfModules];
    for (int i = 0; i < numberOfModules; i++) {
      fieldRelativeModulePosesOfPreviousPose[i] =
          previousRobotPose.transformBy(
              new Transform2d(robotRelativeModuleOffsets[i], Rotation2d.kZero));
    }

    // Also get the module displacements from the previous wheel positions to the current wheel
    // positions
    Translation2d[] moduleDisplacements = new Translation2d[numberOfModules];
    for (int i = 0; i < numberOfModules; i++) {
      moduleDisplacements[i] =
          getModuleDisplacement(
              previousWheelPositions[i].angle.getRadians(),
              previousWheelPositions[i].distanceMeters,
              currentWheelPositions[i].angle.getRadians(),
              currentWheelPositions[i].distanceMeters);
    }

    // Next, add the module displacements to the field relative module poses
    Translation2d[] fieldRelativeModuleDisplacements = new Translation2d[numberOfModules];
    for (int i = 0; i < numberOfModules; i++) {
      fieldRelativeModuleDisplacements[i] =
          fieldRelativeModulePosesOfPreviousPose[i]
              .transformBy(new Transform2d(moduleDisplacements[i], Rotation2d.kZero))
              .getTranslation();
    }

    // Finally, average the module displacements and return the new pose
    Translation2d sumOfFieldRelativeModuleDisplacements = new Translation2d();
    for (int i = 0; i < numberOfModules; i++) {
      sumOfFieldRelativeModuleDisplacements =
          sumOfFieldRelativeModuleDisplacements.plus(fieldRelativeModuleDisplacements[i]);
    }
    double updatedPoseX = sumOfFieldRelativeModuleDisplacements.getX() / numberOfModules;
    double updatedPoseY = sumOfFieldRelativeModuleDisplacements.getY() / numberOfModules;
    var updatedPose = new Pose2d(updatedPoseX, updatedPoseY, currentGyroAngle);

    // Logging
    DogLog.log("Odometry/PreviousPose", previousRobotPose);
    DogLog.log("Odometry/PreviousWheelPositions", previousWheelPositions);
    DogLog.log("Odometry/CurrentWheelPositions", currentWheelPositions);
    DogLog.log("Odometry/ModuleDisplacements", moduleDisplacements);
    DogLog.log(
        "Odometry/FieldRelativeModuleDisplacements/FrontLeft",
        new Pose2d(fieldRelativeModuleDisplacements[0], currentWheelPositions[0].angle));
    DogLog.log(
        "Odometry/FieldRelativeModuleDisplacements/FrontRight",
        new Pose2d(fieldRelativeModuleDisplacements[1], currentWheelPositions[1].angle));
    DogLog.log(
        "Odometry/FieldRelativeModuleDisplacements/BackLeft",
        new Pose2d(fieldRelativeModuleDisplacements[2], currentWheelPositions[2].angle));
    DogLog.log(
        "Odometry/FieldRelativeModuleDisplacements/BackRight",
        new Pose2d(fieldRelativeModuleDisplacements[3], currentWheelPositions[3].angle));

    // After calculations, but before the next loop, update the previous pose & wheel positions to
    // the
    // current ones
    previousRobotPose = updatedPose;
    updatePreviousWheelPositions(currentWheelPositions);

    return updatedPose;
  }

  void updatePreviousWheelPositions(SwerveModulePosition[] currentWheelPositions) {
    for (int i = 0; i < numberOfModules; i++) {
      previousWheelPositions[i] = currentWheelPositions[i];
    }
  }
}
