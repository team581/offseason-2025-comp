package frc.robot.odometry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class OdometryTest {

    @Test
    void sampleOdometryTest() {
      // INITIALIZE CUSTOM ODOMETRY OBJECT
      var customOdometry =
          new CustomOdometry(
              new SwerveDriveKinematics(
                  new Translation2d(12.0, 12.0),
                  new Translation2d(12.0, -12.0),
                  new Translation2d(-12.0, 12.0),
                  new Translation2d(-12.0, -12.0)),
              new Rotation2d(),
              new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
              });

      // TEST VALUES BELOW
      customOdometry.setPreviousRobotPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
      customOdometry.unitTestUpdatePreviousWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(0.0, new Rotation2d(0.0)),
            new SwerveModulePosition(0.0, new Rotation2d(0.0)),
            new SwerveModulePosition(0.0, new Rotation2d(0.0)),
            new SwerveModulePosition(0.0, new Rotation2d(0.0))
          });

      var actual =
          customOdometry.update(
              new Rotation2d(0.0),
              new SwerveModulePosition[] {
                new SwerveModulePosition(0.0, new Rotation2d(0.0)),
                new SwerveModulePosition(0.0, new Rotation2d(0.0)),
                new SwerveModulePosition(0.0, new Rotation2d(0.0)),
                new SwerveModulePosition(0.0, new Rotation2d(0.0))
              });

      assertEquals(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)), actual);
    }

    @Test
    void straightLineDrive() {
      // INITIALIZE CUSTOM ODOMETRY OBJECT
      var customOdometry =
          new CustomOdometry(
              new SwerveDriveKinematics(
                  new Translation2d(12.0, 12.0),
                  new Translation2d(12.0, -12.0),
                  new Translation2d(-12.0, 12.0),
                  new Translation2d(-12.0, -12.0)),
              new Rotation2d(),
              new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
              });

      // TEST VALUES BELOW
      customOdometry.setPreviousRobotPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
      customOdometry.unitTestUpdatePreviousWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(0.0, new Rotation2d(0.0)),
            new SwerveModulePosition(0.0, new Rotation2d(0.0)),
            new SwerveModulePosition(0.0, new Rotation2d(0.0)),
            new SwerveModulePosition(0.0, new Rotation2d(0.0))
          });

      var actual =
          customOdometry.update(
              new Rotation2d(0.0),
              new SwerveModulePosition[] {
                new SwerveModulePosition(5.0, new Rotation2d(0.0)),
                new SwerveModulePosition(5.0, new Rotation2d(0.0)),
                new SwerveModulePosition(5.0, new Rotation2d(0.0)),
                new SwerveModulePosition(5.0, new Rotation2d(0.0))
              });

      assertEquals(new Pose2d(new Translation2d(5.0, 0.0), new Rotation2d(0.0)), actual);
    }

    // FOR ALL TESTS BELOW:
    // -The name indicates modules start direction to end direction, in relation to the unit
    // circle.
    // For example Test upToRight() indicates starting at pi radians(pointing up on unit circle),
    // then
    // travelling on an arc ending at pi / 2 radians(pointing right on unit circle)
    // -Tests will have the previous robot pose at (0, 0, 0) for simplicity
    // -Tests will have modules end distance as 2.0 * pi / 4.0 to represent a fourth of the
    // circumfrence of a circle with raius 1. This is for simplicity so that all end points will
    // be
    // (±1, ±1)

    @Test
    void upToRight() {
      // INITIALIZE CUSTOM ODOMETRY OBJECT
      var customOdometry =
          new CustomOdometry(
              new SwerveDriveKinematics(
                  new Translation2d(12.0, 12.0),
                  new Translation2d(12.0, -12.0),
                  new Translation2d(-12.0, 12.0),
                  new Translation2d(-12.0, -12.0)),
              new Rotation2d(),
              new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
              });

      // TEST VALUES BELOW
      customOdometry.setPreviousRobotPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
      customOdometry.unitTestUpdatePreviousWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(0.0, new Rotation2d(Math.PI)),
            new SwerveModulePosition(0.0, new Rotation2d(Math.PI)),
            new SwerveModulePosition(0.0, new Rotation2d(Math.PI)),
            new SwerveModulePosition(0.0, new Rotation2d(Math.PI))
          });

      var actual =
          customOdometry.update(
              new Rotation2d(0.0),
              new SwerveModulePosition[] {
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0))
              });

      assertEquals(new Pose2d(new Translation2d(1.0, 1.0), new Rotation2d(0.0)), actual);
    }

    @Test
    void rightToUp() {
      // INITIALIZE CUSTOM ODOMETRY OBJECT
      var customOdometry =
          new CustomOdometry(
              new SwerveDriveKinematics(
                  new Translation2d(12.0, 12.0),
                  new Translation2d(12.0, -12.0),
                  new Translation2d(-12.0, 12.0),
                  new Translation2d(-12.0, -12.0)),
              new Rotation2d(),
              new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
              });

      // TEST VALUES BELOW
      customOdometry.setPreviousRobotPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
      customOdometry.unitTestUpdatePreviousWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
            new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
            new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
            new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0))
          });

      var actual =
          customOdometry.update(
              new Rotation2d(0.0),
              new SwerveModulePosition[] {
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(2.0 * Math.PI)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(2.0 * Math.PI)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(2.0 * Math.PI)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(2.0 * Math.PI))
              });

      assertEquals(new Pose2d(new Translation2d(1.0, 1.0), new Rotation2d(0.0)), actual);
    }

    @Test
    void upToLeft() {
      // INITIALIZE CUSTOM ODOMETRY OBJECT
      var customOdometry =
          new CustomOdometry(
              new SwerveDriveKinematics(
                  new Translation2d(12.0, 12.0),
                  new Translation2d(12.0, -12.0),
                  new Translation2d(-12.0, 12.0),
                  new Translation2d(-12.0, -12.0)),
              new Rotation2d(),
              new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
              });

      // TEST VALUES BELOW
      customOdometry.setPreviousRobotPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
      customOdometry.unitTestUpdatePreviousWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(0.0, new Rotation2d(0.0)),
            new SwerveModulePosition(0.0, new Rotation2d(0.0)),
            new SwerveModulePosition(0.0, new Rotation2d(0.0)),
            new SwerveModulePosition(0.0, new Rotation2d(0.0))
          });

      var actual =
          customOdometry.update(
              new Rotation2d(0.0),
              new SwerveModulePosition[] {
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI / 2.0))
              });

      assertEquals(new Pose2d(new Translation2d(-1.0, 1.0), new Rotation2d(0.0)), actual);
    }

    @Test
    void leftToUp() {
      // INITIALIZE CUSTOM ODOMETRY OBJECT
      var customOdometry =
          new CustomOdometry(
              new SwerveDriveKinematics(
                  new Translation2d(12.0, 12.0),
                  new Translation2d(12.0, -12.0),
                  new Translation2d(-12.0, 12.0),
                  new Translation2d(-12.0, -12.0)),
              new Rotation2d(),
              new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
              });

      // TEST VALUES BELOW
      customOdometry.setPreviousRobotPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
      customOdometry.unitTestUpdatePreviousWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
            new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
            new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
            new SwerveModulePosition(0.0, new Rotation2d((3.0 * Math.PI) / 2.0))
          });

      var actual =
          customOdometry.update(
              new Rotation2d(0.0),
              new SwerveModulePosition[] {
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI))
              });

      assertEquals(new Pose2d(new Translation2d(-1.0, 1.0), new Rotation2d(0.0)), actual);
    }

    @Test
    void downToRight() {
      // INITIALIZE CUSTOM ODOMETRY OBJECT
      var customOdometry =
          new CustomOdometry(
              new SwerveDriveKinematics(
                  new Translation2d(12.0, 12.0),
                  new Translation2d(12.0, -12.0),
                  new Translation2d(-12.0, 12.0),
                  new Translation2d(-12.0, -12.0)),
              new Rotation2d(),
              new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
              });

      // TEST VALUES BELOW
      customOdometry.setPreviousRobotPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
      customOdometry.unitTestUpdatePreviousWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(0.0, new Rotation2d(Math.PI)),
            new SwerveModulePosition(0.0, new Rotation2d(Math.PI)),
            new SwerveModulePosition(0.0, new Rotation2d(Math.PI)),
            new SwerveModulePosition(0.0, new Rotation2d(Math.PI))
          });

      var actual =
          customOdometry.update(
              new Rotation2d(0.0),
              new SwerveModulePosition[] {
                new SwerveModulePosition(
                    (2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
                new SwerveModulePosition(
                    (2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
                new SwerveModulePosition(
                    (2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) /
  2.0))
              });

      assertEquals(new Pose2d(new Translation2d(1.0, -1.0), new Rotation2d(0.0)), actual);
    }

    @Test
    void rightToDown() {
      // INITIALIZE CUSTOM ODOMETRY OBJECT
      var customOdometry =
          new CustomOdometry(
              new SwerveDriveKinematics(
                  new Translation2d(12.0, 12.0),
                  new Translation2d(12.0, -12.0),
                  new Translation2d(-12.0, 12.0),
                  new Translation2d(-12.0, -12.0)),
              new Rotation2d(),
              new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
              });

      // TEST VALUES BELOW
      customOdometry.setPreviousRobotPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
      customOdometry.unitTestUpdatePreviousWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
            new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
            new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
            new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0))
          });

      var actual =
          customOdometry.update(
              new Rotation2d(0.0),
              new SwerveModulePosition[] {
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(0.0)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(0.0)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(0.0)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(0.0))
              });

      assertEquals(new Pose2d(new Translation2d(1.0, -1.0), new Rotation2d(0.0)), actual);
    }

    @Test
    void downToLeft() {
      // INITIALIZE CUSTOM ODOMETRY OBJECT
      var customOdometry =
          new CustomOdometry(
              new SwerveDriveKinematics(
                  new Translation2d(12.0, 12.0),
                  new Translation2d(12.0, -12.0),
                  new Translation2d(-12.0, 12.0),
                  new Translation2d(-12.0, -12.0)),
              new Rotation2d(),
              new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
              });

      // TEST VALUES BELOW
      customOdometry.setPreviousRobotPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
      customOdometry.unitTestUpdatePreviousWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(0.0, new Rotation2d(2.0 * Math.PI)),
            new SwerveModulePosition(0.0, new Rotation2d(2.0 * Math.PI)),
            new SwerveModulePosition(0.0, new Rotation2d(2.0 * Math.PI)),
            new SwerveModulePosition(0.0, new Rotation2d(2.0 * Math.PI))
          });

      var actual =
          customOdometry.update(
              new Rotation2d(0.0),
              new SwerveModulePosition[] {
                new SwerveModulePosition(
                    (2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
                new SwerveModulePosition(
                    (2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
                new SwerveModulePosition(
                    (2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) / 2.0)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d((3.0 * Math.PI) /
  2.0))
              });

      assertEquals(new Pose2d(new Translation2d(-1.0, -1.0), new Rotation2d(0.0)), actual);
    }

    @Test
    void leftToDown() {
      // INITIALIZE CUSTOM ODOMETRY OBJECT
      var customOdometry =
          new CustomOdometry(
              new SwerveDriveKinematics(
                  new Translation2d(12.0, 12.0),
                  new Translation2d(12.0, -12.0),
                  new Translation2d(-12.0, 12.0),
                  new Translation2d(-12.0, -12.0)),
              new Rotation2d(),
              new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
              });

      // TEST VALUES BELOW
      customOdometry.setPreviousRobotPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
      customOdometry.unitTestUpdatePreviousWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
            new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
            new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0)),
            new SwerveModulePosition(0.0, new Rotation2d(Math.PI / 2.0))
          });

      var actual =
          customOdometry.update(
              new Rotation2d(0.0),
              new SwerveModulePosition[] {
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI)),
                new SwerveModulePosition((2.0 * Math.PI) / 4.0, new Rotation2d(Math.PI))
              });

      assertEquals(new Pose2d(new Translation2d(-1.0, -1.0), new Rotation2d(0.0)), actual);
    }
}
