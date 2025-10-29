package frc.robot.odometry;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class CustomOdometry extends SwerveDriveOdometry {
  private final Translation2d[] robotRelativeModuleOffsets = new Translation2d[4];

  private Pose2d previousRobotPose = new Pose2d();
  private final SwerveModulePosition[] previousWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  public CustomOdometry(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions) {
    super(kinematics, gyroAngle, modulePositions);

    for (int i = 0; i < 4; i++) {
      robotRelativeModuleOffsets[i] =
          new Translation2d(kinematics.getModules()[i].getX(), kinematics.getModules()[i].getY());
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
    Pose2d[] fieldRelativeModulePosesOfPreviousPose = {
      previousRobotPose.transformBy(
          new Transform2d(robotRelativeModuleOffsets[0], new Rotation2d(0.0))),
      previousRobotPose.transformBy(
          new Transform2d(robotRelativeModuleOffsets[1], new Rotation2d(0.0))),
      previousRobotPose.transformBy(
          new Transform2d(robotRelativeModuleOffsets[2], new Rotation2d(0.0))),
      previousRobotPose.transformBy(
          new Transform2d(robotRelativeModuleOffsets[3], new Rotation2d(0.0)))
    };

    // Also get the module displacements from the previous wheel positions to the current wheel
    // positions
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

    // Next, add the module displacements to the field relative module poses
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

    // Finally, average the module displacements and return the new pose
    Translation2d sumOfFieldRelativeModuleDisplacements =
        fieldRelativeModuleDisplacements[0]
            .plus(fieldRelativeModuleDisplacements[1])
            .plus(fieldRelativeModuleDisplacements[2])
            .plus(fieldRelativeModuleDisplacements[3]);
    double updatedPoseX = sumOfFieldRelativeModuleDisplacements.getX() / 4.0;
    double updatedPoseY = sumOfFieldRelativeModuleDisplacements.getY() / 4.0;
    var updatedPose = new Pose2d(updatedPoseX, updatedPoseY, currentGyroAngle);

    // Logging
    DogLog.log("Odometry/PreviousPose", previousRobotPose);
    DogLog.log("Odometry/PreviousWheelPositions", previousWheelPositions);
    DogLog.log("Odometry/CurrentWheelPositions", currentWheelPositions);
    DogLog.log("Odometry/ModuleDisplacements", moduleDisplacements);
    DogLog.log(
        "Odometry/FieldRelativeModuleDisplacements/FrontLeft",
        new Pose2d(
            fieldRelativeModuleDisplacements[0],
            new Rotation2d(currentWheelPositions[0].angle.getRadians())));
    DogLog.log(
        "Odometry/FieldRelativeModuleDisplacements/FrontRight",
        new Pose2d(
            fieldRelativeModuleDisplacements[1],
            new Rotation2d(currentWheelPositions[1].angle.getRadians())));
    DogLog.log(
        "Odometry/FieldRelativeModuleDisplacements/BackLeft",
        new Pose2d(
            fieldRelativeModuleDisplacements[2],
            new Rotation2d(currentWheelPositions[2].angle.getRadians())));
    DogLog.log(
        "Odometry/FieldRelativeModuleDisplacements/BackRight",
        new Pose2d(
            fieldRelativeModuleDisplacements[3],
            new Rotation2d(currentWheelPositions[3].angle.getRadians())));

    // After calculations, but before the next loop, update the previous pose & wheel positions to
    // the
    // current ones
    previousRobotPose = updatedPose;
    updatePreviousWheelPositions(currentWheelPositions);

    return updatedPose;
  }

  private void updatePreviousWheelPositions(SwerveModulePosition[] currentWheelPositions) {
    for (int i = 0; i < 4; i++) {
      previousWheelPositions[i] = currentWheelPositions[i];
    }
  }

  // PUBLIC FUNCTION FOR UNIT TESTS ONLY
  public void unitTestUpdatePreviousWheelPositions(SwerveModulePosition[] currentWheelPositions) {
    for (int i = 0; i < 4; i++) {
      previousWheelPositions[i] = currentWheelPositions[i];
    }
  }
}
