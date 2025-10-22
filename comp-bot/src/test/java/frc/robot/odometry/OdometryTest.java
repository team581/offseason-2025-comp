package frc.robot.odometry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.junit.jupiter.api.Test;

public class OdometryTest {
  // @Test
  // void straightLineDrive() {
  //   var odometry = new CustomOdometry();

  //   var actual =
  //       odometry.update(
  //           new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(0.0, new Rotation2d(0.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(0.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(0.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(0.0))
  //           },
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(5.0, new Rotation2d(0.0)),
  //             new SwerveModulePosition(5.0, new Rotation2d(0.0)),
  //             new SwerveModulePosition(5.0, new Rotation2d(0.0)),
  //             new SwerveModulePosition(5.0, new Rotation2d(0.0))
  //           },
  //           new Rotation2d(0.0));

  //   assertEquals(new Pose2d(new Translation2d(5.0, 0.0), new Rotation2d(0.0)), actual);
  // }

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

  // @Test
  // void frontToLeftCurveLineDrive() {
  //   var odometry = new CustomOdometry();

  //   var actual =
  //       odometry.update(
  //           new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(0.0, new Rotation2d(0.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(0.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(0.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(0.0))
  //           },
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0))
  //           },
  //           new Rotation2d(0.0));

  //   assertEquals(new Pose2d(new Translation2d(-1.0, 1.0), new Rotation2d(0.0)), actual);
  // }

  // @Test
  // void leftToFrontCurveLineDrive() {
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
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI))
  //           },
  //           new Rotation2d(0.0));

  //   assertEquals(new Pose2d(new Translation2d(-1.0, 1.0), new Rotation2d(0.0)), actual);
  // }

  // @Test
  // void backToRightCurveLineDrive() {
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
  //             new SwerveModulePosition(
  //                 (2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition(
  //                 (2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition(
  //                 (2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) / 2.0))
  //           },
  //           new Rotation2d(0.0));

  //   assertEquals(new Pose2d(new Translation2d(1.0, -1.0), new Rotation2d(0.0)), actual);
  // }

  // @Test
  // void rightToBackCurveLineDrive() {
  //   var odometry = new CustomOdometry();

  //   var actual =
  //       odometry.update(
  //           new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0))
  //           },
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(0.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(0.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(0.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(0.0))
  //           },
  //           new Rotation2d(0.0));

  //   assertEquals(new Pose2d(new Translation2d(1.0, -1.0), new Rotation2d(0.0)), actual);
  // }

  // @Test
  // void backToLeftCurveLineDrive() {
  //   var odometry = new CustomOdometry();

  //   var actual =
  //       odometry.update(
  //           new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(0.0, new Rotation2d(2.0 * Math.PI)),
  //             new SwerveModulePosition(0.0, new Rotation2d(2.0 * Math.PI)),
  //             new SwerveModulePosition(0.0, new Rotation2d(2.0 * Math.PI)),
  //             new SwerveModulePosition(0.0, new Rotation2d(2.0 * Math.PI))
  //           },
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(
  //                 (2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition(
  //                 (2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition(
  //                 (2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) / 2.0))
  //           },
  //           new Rotation2d(0.0));

  //   assertEquals(new Pose2d(new Translation2d(-1.0, -1.0), new Rotation2d(0.0)), actual);
  // }

  // @Test
  // void leftToBackCurveLineDrive() {
  //   var odometry = new CustomOdometry();

  //   var actual =
  //       odometry.update(
  //           new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0))
  //           },
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI))
  //           },
  //           new Rotation2d(0.0));

  //   assertEquals(new Pose2d(new Translation2d(-1.0, -1.0), new Rotation2d(0.0)), actual);
  // }

  // @Test
  // void negativeDistanceFrontToRightCurveLineDrive() {
  //   var odometry = new CustomOdometry();

  //   var actual =
  //       odometry.update(
  //           new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI))
  //           },
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0))
  //           },
  //           new Rotation2d(0.0));

  //   assertEquals(new Pose2d(new Translation2d(1.0, 1.0), new Rotation2d(0.0)), actual);
  // }

  // @Test
  // void negativeDistanceRightToFrontCurveLineDrive() {
  //   var odometry = new CustomOdometry();

  //   var actual =
  //       odometry.update(
  //           new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(
  //                 (2.0 * Math.PI) / 4.0 + 5.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition(
  //                 (2.0 * Math.PI) / 4.0 + 5.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition(
  //                 (2.0 * Math.PI) / 4.0 + 5.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition(
  //                 (2.0 * Math.PI) / 4.0 + 5.0, new Rotation2d((3.0 * Math.PI) / 2.0))
  //           },
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(0.0 + 5.0, new Rotation2d(2.0 * Math.PI)),
  //             new SwerveModulePosition(0.0 + 5.0, new Rotation2d(2.0 * Math.PI)),
  //             new SwerveModulePosition(0.0 + 5.0, new Rotation2d(2.0 * Math.PI)),
  //             new SwerveModulePosition(0.0 + 5.0, new Rotation2d(2.0 * Math.PI))
  //           },
  //           new Rotation2d(0.0));

  //   assertEquals(new Pose2d(new Translation2d(1.0, 1.0), new Rotation2d(0.0)), actual);
  // }

  // @Test
  // void negativeDistanceFrontToLeftCurveLineDrive() {
  //   var odometry = new CustomOdometry();

  //   var actual =
  //       odometry.update(
  //           new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(0.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(0.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(0.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(0.0))
  //           },
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0))
  //           },
  //           new Rotation2d(0.0));

  //   assertEquals(new Pose2d(new Translation2d(-1.0, 1.0), new Rotation2d(0.0)), actual);
  // }

  // @Test
  // void negativeDistanceLeftToFrontCurveLineDrive() {
  //   var odometry = new CustomOdometry();

  //   var actual =
  //       odometry.update(
  //           new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(
  //                 (2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition(
  //                 (2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition(
  //                 (2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) / 2.0))
  //           },
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI))
  //           },
  //           new Rotation2d(0.0));

  //   assertEquals(new Pose2d(new Translation2d(-1.0, 1.0), new Rotation2d(0.0)), actual);
  // }

  // @Test
  // void negativeDistanceBackToRightCurveLineDrive() {
  //   var odometry = new CustomOdometry();

  //   var actual =
  //       odometry.update(
  //           new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI))
  //           },
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0))
  //           },
  //           new Rotation2d(0.0));

  //   assertEquals(new Pose2d(new Translation2d(1.0, -1.0), new Rotation2d(0.0)), actual);
  // }

  // @Test
  // void negativeDistanceRightToBackCurveLineDrive() {
  //   var odometry = new CustomOdometry();

  //   var actual =
  //       odometry.update(
  //           new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0))
  //           },
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(0.0, new Rotation2d(0.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(0.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(0.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d(0.0))
  //           },
  //           new Rotation2d(0.0));

  //   assertEquals(new Pose2d(new Translation2d(1.0, -1.0), new Rotation2d(0.0)), actual);
  // }

  // @Test
  // void negativeDistanceBackToLeftCurveLineDrive() {
  //   var odometry = new CustomOdometry();

  //   var actual =
  //       odometry.update(
  //           new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(2.0 * Math.PI)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(2.0 * Math.PI)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(2.0 * Math.PI)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(2.0 * Math.PI))
  //           },
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
  //             new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0))
  //           },
  //           new Rotation2d(0.0));

  //   assertEquals(new Pose2d(new Translation2d(-1.0, -1.0), new Rotation2d(0.0)), actual);
  // }

  // @Test
  // void negativeDistanceLeftToBackCurveLineDrive() {
  //   var odometry = new CustomOdometry();

  //   var actual =
  //       odometry.update(
  //           new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0)),
  //             new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0))
  //           },
  //           new SwerveModulePosition[] {
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI)),
  //             new SwerveModulePosition(0.0, new Rotation2d(Math.PI))
  //           },
  //           new Rotation2d(0.0));

  //   assertEquals(new Pose2d(new Translation2d(-1.0, -1.0), new Rotation2d(0.0)), actual);
  // }
}
