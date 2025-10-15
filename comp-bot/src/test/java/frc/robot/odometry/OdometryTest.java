package frc.robot.odometry;

import static edu.wpi.first.units.Units.Inches;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.junit.jupiter.api.Test;

public class OdometryTest {
  private static final Translation2d[] robotRelativeModulePositions = {
    new Translation2d(Inches.of(12), Inches.of(12)), // front-left
    new Translation2d(Inches.of(12), Inches.of(-12)), // front-right
    new Translation2d(Inches.of(-12), Inches.of(12)), // back-left
    new Translation2d(Inches.of(-12), Inches.of(-12)) // back-right
  };

  private static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(robotRelativeModulePositions);

  @Test
  void testStraightLineDrive() {
    var odometry = new CustomOdometry(Rotation2d.kZero, Pose2d.kZero);

    var actual =
        odometry.update(
            new SwerveModulePosition[] {
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(0))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(0))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(0))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(0)))
            },
            new SwerveModulePosition[] {
              new SwerveModulePosition(5.0, new Rotation2d(Math.toRadians(0))),
              new SwerveModulePosition(5.0, new Rotation2d(Math.toRadians(0))),
              new SwerveModulePosition(5.0, new Rotation2d(Math.toRadians(0))),
              new SwerveModulePosition(5.0, new Rotation2d(Math.toRadians(0)))
            });

    assertEquals(new Pose2d(new Translation2d(5.0, 0.0), new Rotation2d(0.0)), actual);

  }

  // TODO: make this test actually model a curve
  @Test
  void curveLineDrive() {
    var odometry = new CustomOdometry(Rotation2d.kZero, Pose2d.kZero);

    var actual =
        odometry.update(
            new SwerveModulePosition[] {
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(0))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(0))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(0))),
              new SwerveModulePosition(0.0, new Rotation2d(Math.toRadians(0)))
            },
            new SwerveModulePosition[] {
              new SwerveModulePosition(5.0, new Rotation2d(Math.toRadians(0))),
              new SwerveModulePosition(5.0, new Rotation2d(Math.toRadians(0))),
              new SwerveModulePosition(5.0, new Rotation2d(Math.toRadians(0))),
              new SwerveModulePosition(5.0, new Rotation2d(Math.toRadians(0)))
            });

    assertEquals(new Pose2d(new Translation2d(5.0, 0.0), new Rotation2d(0.0)), actual);
  }
}
