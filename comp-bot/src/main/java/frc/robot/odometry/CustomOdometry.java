package frc.robot.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

// TODO: 1) Get odometry in a usable state, then implement fixes for 2) Wheel slip, 3) Accounting
// for downtime when wheels turn
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
    return Pose2d.kZero;
  }
}
