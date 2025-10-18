package frc.robot.odometry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.junit.jupiter.api.Test;

public class OdometryTest {
// TODO: Make all test formats & units consistent with the first 3 tests

  @Test
  void testStraightLineDrive() {
    var odometry = new CustomOdometry(Rotation2d.kZero, Pose2d.kZero);

    var actual =
        odometry.update(
            new SwerveModulePosition[] {
              new SwerveModulePosition(0.0, new Rotation2d(0.0)),
              new SwerveModulePosition(0.0, new Rotation2d(0.0)),
              new SwerveModulePosition(0.0, new Rotation2d(0.0)),
              new SwerveModulePosition(0.0, new Rotation2d(0.0))
            },
            new SwerveModulePosition[] {
              new SwerveModulePosition(5.0, new Rotation2d(0.0)),
              new SwerveModulePosition(5.0, new Rotation2d(0.0)),
              new SwerveModulePosition(5.0, new Rotation2d(0.0)),
              new SwerveModulePosition(5.0, new Rotation2d(0.0))
            });

    assertEquals(new Pose2d(new Translation2d(5.0, 0.0), new Rotation2d(0.0)), actual);
  }

  @Test
  void frontToRightCurveLineDrive() {
    var odometry = new CustomOdometry(Rotation2d.kZero, Pose2d.kZero);

    var actual =
        odometry.update(
            new SwerveModulePosition[] {
              new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
              new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
              new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
              new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0))
            },
            new SwerveModulePosition[] {
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(0.0)),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(0.0)),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(0.0)),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(0.0))
            });

    assertEquals(new Pose2d(new Translation2d(1.0, 1.0), new Rotation2d(0.0)), actual);
  }

  @Test
  void rightToFrontCurveLineDrive() {
    var odometry = new CustomOdometry(Rotation2d.kZero, Pose2d.kZero);

    var actual =
        odometry.update(
            new SwerveModulePosition[] {
              new SwerveModulePosition(0.0, new Rotation2d(0.0)),
              new SwerveModulePosition(0.0, new Rotation2d(0.0)),
              new SwerveModulePosition(0.0, new Rotation2d(0.0)),
              new SwerveModulePosition(0.0, new Rotation2d(0.0))
            },
            new SwerveModulePosition[] {
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2)),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2)),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2)),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2))
            });

    assertEquals(new Pose2d(new Translation2d(1.0, 1.0), new Rotation2d(Math.PI / 2)), actual);
  }

  @Test
  void frontToLeftCurveLineDrive() {
    var odometry = new CustomOdometry(Rotation2d.kZero, Pose2d.kZero);

    var actual =
        odometry.update(
            new SwerveModulePosition[] {
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(0.0))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(0.0))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(0.0))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(0.0)))
            },
            new SwerveModulePosition[] {
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(-90))),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(-90))),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(-90))),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(-90)))
            });

    assertEquals(new Pose2d(new Translation2d(-1.0, 1.0), new Rotation2d(-90.0)), actual);
  }

  @Test
  void leftToFrontCurveLineDrive() {
    var odometry = new CustomOdometry(Rotation2d.kZero, Pose2d.kZero);

    var actual =
        odometry.update(
            new SwerveModulePosition[] {
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(-90))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(-90))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(-90))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(-90)))
            },
            new SwerveModulePosition[] {
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(0.0))),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(0.0))),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(0.0))),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(0.0)))
            });

    assertEquals(new Pose2d(new Translation2d(-1.0, 1.0), new Rotation2d(0.0)), actual);
  }

  @Test
  void backToRightCurveLineDrive() {
    var odometry = new CustomOdometry(Rotation2d.kZero, Pose2d.kZero);

    var actual =
        odometry.update(
            new SwerveModulePosition[] {
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(180))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(180))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(180))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(180)))
            },
            new SwerveModulePosition[] {
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(90))),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(90))),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(90))),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(90)))
            });

    assertEquals(new Pose2d(new Translation2d(1.0, -1.0), new Rotation2d(90.0)), actual);
  }

  @Test
  void rightToBackCurveLineDrive() {
    var odometry = new CustomOdometry(Rotation2d.kZero, Pose2d.kZero);

    var actual =
        odometry.update(
            new SwerveModulePosition[] {
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(90))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(90))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(90))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(90)))
            },
            new SwerveModulePosition[] {
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(180))),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(180))),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(180))),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(180)))
            });

    assertEquals(new Pose2d(new Translation2d(1.0, -1.0), new Rotation2d(180.0)), actual);
  }

  @Test
  void backToLeftCurveLineDrive() {
    var odometry = new CustomOdometry(Rotation2d.kZero, Pose2d.kZero);

    var actual =
        odometry.update(
            new SwerveModulePosition[] {
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(-180))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(-180))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(-180))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(-180)))
            },
            new SwerveModulePosition[] {
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(-90))),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(-90))),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(-90))),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(-90)))
            });

    assertEquals(new Pose2d(new Translation2d(-1.0, -1.0), new Rotation2d(-90.0)), actual);
  }

  @Test
  void leftToBackCurveLineDrive() {
    var odometry = new CustomOdometry(Rotation2d.kZero, Pose2d.kZero);

    var actual =
        odometry.update(
            new SwerveModulePosition[] {
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(-90))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(-90))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(-90))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(-90)))
            },
            new SwerveModulePosition[] {
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(-180))),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(-180))),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(-180))),
              new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.toRadians(-180)))
            });

    assertEquals(new Pose2d(new Translation2d(-1.0, -1.0), new Rotation2d(-180.0)), actual);
  }
}
