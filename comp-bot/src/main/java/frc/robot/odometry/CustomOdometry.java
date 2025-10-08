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

  public Pose2d update(
      SwerveModulePosition[] previousWheelPositions, SwerveModulePosition[] currentWheelPositions) {
    double frontLeftModuledisplacement =
        previousWheelPositions[0].distanceMeters + currentWheelPositions[0].distanceMeters;

    return new Pose2d(new Translation2d(), gyroAngle);
  }
}
