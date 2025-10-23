package frc.robot.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class OdometryTest {

  @Test
  void straightLineDrive() {
  var customOdometry = new CustomOdometry(new SwerveDriveKinematics(), new Rotation2d(), new SwerveModulePosition[4]);

  customOdometry.setPreviousRobotPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
  customOdometry.updatePreviousWheelPositions(new SwerveModulePosition[] {
    new SwerveModulePosition(0.0, new Rotation2d(0.0)),
    new SwerveModulePosition(0.0, new Rotation2d(0.0)),
    new SwerveModulePosition(0.0, new Rotation2d(0.0)),
    new SwerveModulePosition(0.0, new Rotation2d(0.0))
  });

  var actual = customOdometry.update(0.0, new SwerveModulePosition[] {
    new SwerveModulePosition(0.0, new Rotation2d(0.0)),
    new SwerveModulePosition(0.0, new Rotation2d(0.0)),
    new SwerveModulePosition(0.0, new Rotation2d(0.0)),
    new SwerveModulePosition(0.0, new Rotation2d(0.0))
  });

    assertEquals(new Pose2d(new Translation2d(5.0, 0.0), new Rotation2d(0.0)), actual);
  }

  // @Test
  // void negativeStraightLineDrive() {
  //   var odometry = new CustomOdometry();

  //   var actual =
  //       odometry.update(
  //           new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(5.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition(5.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition(5.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition(5.0, new Rotation2d(Math.PI))
  //           },
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI))
  //           },
  //           new Rotation2d(0.0));

  //   assertEquals(new Pose2d(new Translation2d(5.0, 0.0), new Rotation2d(0.0)), actual);
  // }

  // @Test
  // void frontToRightCurveLineDrive() {
  //   var odometry = new CustomOdometry();

  //   var actual =
  //       odometry.update(
  //           new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI))
  //           },
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0))
  //           },
  //           new Rotation2d(0.0));

  //   assertEquals(new Pose2d(new Translation2d(1.0, 1.0), new Rotation2d(0.0)), actual);
  // }

  // @Test
  // void rightToFrontCurveLineDrive() {
  //   var odometry = new CustomOdometry();

  //   var actual =
  //       odometry.update(
  //           new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0))
  //           },
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(2.0 * Math.PI)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(2.0 * Math.PI)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(2.0 * Math.PI)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(2.0 * Math.PI))
  //           },
  //           new Rotation2d(0.0));

  //   assertEquals(new Pose2d(new Translation2d(1.0, 1.0), new Rotation2d(0.0)), actual);
  // }
}
