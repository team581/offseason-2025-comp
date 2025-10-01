package frc.robot.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class CustomOdometry extends Odometry<SwerveModulePosition[]> {

  public CustomOdometry(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] wheelPositions,
      Pose2d initialPoseMeters) {
    super(kinematics, gyroAngle, wheelPositions, initialPoseMeters);
  }

  @Override
  public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
    Pose2d updatedPose = new Pose2d();
    return updatedPose;
  }
}
