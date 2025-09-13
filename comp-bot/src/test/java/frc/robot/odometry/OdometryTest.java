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
    var start =
        new SwerveModulePosition[] {
          new SwerveModulePosition(0.0, new Rotation2d(0.0)), // front-left
          new SwerveModulePosition(0.0, new Rotation2d(0.0)), // front-right
          new SwerveModulePosition(0.0, new Rotation2d(0.0)), // back-left
          new SwerveModulePosition(0.0, new Rotation2d(0.0)) // back-right
        };
    var odometry = new CustomOdometry(kinematics, Rotation2d.kZero, start, Pose2d.kZero);
    var end =
        new SwerveModulePosition[] {
          new SwerveModulePosition(5.0, new Rotation2d(0.0)), // front-left
          new SwerveModulePosition(5.0, new Rotation2d(0.0)), // front-right
          new SwerveModulePosition(5.0, new Rotation2d(0.0)), // back-left
          new SwerveModulePosition(5.0, new Rotation2d(0.0)) // back-right
        };

    odometry.update(Rotation2d.kZero, start);
    var actual = odometry.update(Rotation2d.kZero, end);

    assertEquals(new Pose2d(new Translation2d(5.0, 0.0), new Rotation2d(0.0)), actual);
  }
}
