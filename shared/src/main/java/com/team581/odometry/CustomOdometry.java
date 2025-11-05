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

  private Pose2d robotPose = Pose2d.kZero;
  private final SwerveModulePosition[] previousWheelPositions;

  public CustomOdometry(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions) {
    super(kinematics, gyroAngle, modulePositions);
    numberOfModules = kinematics.getModules().length;
    robotRelativeModuleOffsets = kinematics.getModules();

    previousWheelPositions = new SwerveModulePosition[numberOfModules];
    for (int i = 0; i < numberOfModules; i++) {
      previousWheelPositions[i] = new SwerveModulePosition();
    }
  }

  private static Translation2d getModuleDisplacement(
      SwerveModulePosition previousWheelPosition, SwerveModulePosition currentWheelPosition) {
    // First, calculate difference between previous and current angles and distances
    double angleDifferenceRadians =
        currentWheelPosition.angle.getRadians() - previousWheelPosition.angle.getRadians();
    double arcLength = currentWheelPosition.distanceMeters - previousWheelPosition.distanceMeters;

    // *If angle difference is 0 then we can just use a straight line instead of an arc
    if (angleDifferenceRadians == 0) {
      return new Translation2d(arcLength, currentWheelPosition.angle);
    }

    // Next, calculate radius. Positive = left turn, negative = right turn
    double radius = (arcLength / angleDifferenceRadians);
    DogLog.log("Odometry/GetModuleDisplacement/Radius", radius);

    // Then, calculate the center point of the circle that the arc is a part of, using the previous
    // angle. The previous module translation is (0, 0) because we don't care where it starts, only
    // the displacement. It is also always perpendicular to the preivous angle, with
    // positive/negative radius indicating which side of the module that the circle center will be
    // on
    double circleCenterX = -radius * previousWheelPosition.angle.getSin();
    double circleCenterY = radius * previousWheelPosition.angle.getCos();

    // Finally, calculate the current module translation on the arc and return it as module
    // displacement
    double displacementX = circleCenterX + radius * currentWheelPosition.angle.getSin();
    double displacementY = circleCenterY - radius * currentWheelPosition.angle.getCos();

    return new Translation2d(displacementX, displacementY);
  }

  @Override
  public Pose2d update(Rotation2d currentGyroAngle, SwerveModulePosition[] currentWheelPositions) {
    // First, get the field relative module poses of the previous robot pose, and apply robot
    // relative module offsets
    Pose2d[] fieldRelativeModulePosesOfPreviousPose = new Pose2d[numberOfModules];
    for (int i = 0; i < numberOfModules; i++) {
      fieldRelativeModulePosesOfPreviousPose[i] =
          robotPose.transformBy(new Transform2d(robotRelativeModuleOffsets[i], Rotation2d.kZero));
    }

    // Also get the module displacements from the previous wheel positions to the current wheel
    // positions
    Translation2d[] moduleDisplacements = new Translation2d[numberOfModules];
    for (int i = 0; i < numberOfModules; i++) {
      moduleDisplacements[i] =
          getModuleDisplacement(previousWheelPositions[i], currentWheelPositions[i]);
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
    DogLog.log("Odometry/PreviousPose", robotPose);
    DogLog.log("Odometry/PreviousWheelPositions", previousWheelPositions);
    DogLog.log("Odometry/CurrentWheelPositions", currentWheelPositions);
    DogLog.log("Odometry/ModuleDisplacements", moduleDisplacements);
    for (int i = 0; i < numberOfModules; i++) {
      DogLog.log(
          "Odometry/FieldRelativeModuleDisplacements/" + i,
          new Pose2d(
              fieldRelativeModuleDisplacements[i],
              currentGyroAngle.plus(currentWheelPositions[i].angle)));
    }

    // After calculations, but before the next loop, update the previous pose & wheel positions to
    // the current ones
    robotPose = updatedPose;
    for (int i = 0; i < numberOfModules; i++) {
      previousWheelPositions[i] = currentWheelPositions[i];
    }

    return updatedPose;
  }

  @Override
  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d poseMeters) {
    robotPose = new Pose2d(poseMeters.getTranslation(), gyroAngle);
  }

  @Override
  public void resetPose(Pose2d poseMeters) {
    robotPose = poseMeters;
  }

  @Override
  public void resetTranslation(Translation2d translation) {
    robotPose = new Pose2d(translation, robotPose.getRotation());
  }

  @Override
  public void resetRotation(Rotation2d rotation) {
    robotPose = new Pose2d(robotPose.getTranslation(), rotation);
  }

  @Override
  public Pose2d getPoseMeters() {
    return robotPose;
  }
}
