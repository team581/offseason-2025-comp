package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class MagnetismUtilTest {

  Translation2d getTranslationFromChassisSpeeds(ChassisSpeeds speeds) {
    return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  double getMagnitudeFromChassisSpeeds(ChassisSpeeds speeds) {
    return Math.round(getTranslationFromChassisSpeeds(speeds).getNorm());
  }

  Rotation2d getAngleFromChassisSpeeds(ChassisSpeeds speeds) {
    return getTranslationFromChassisSpeeds(speeds).getAngle();
  }

  @Test
  void testMagnetizedMagnitudeSameAsInputMagnitude() {
    /* Verify input magnitude is the same as the output magnitude. */
    ChassisSpeeds robotSpeed = new ChassisSpeeds(20.56, 11.14, 0.0);
    Pose2d robotPose = Pose2d.kZero;
    Pose2d goalPose = Pose2d.kZero;

    ChassisSpeeds magnetizedSpeed =
        MagnetismUtil.getMagnetizedChassisSpeeds(robotSpeed, robotPose, goalPose);
    Assertions.assertEquals(
        getMagnitudeFromChassisSpeeds(robotSpeed), getMagnitudeFromChassisSpeeds(magnetizedSpeed));
  }

  @Test
  void testMagnetizedAngleBetweenInputAngleAndStraightLineAngle() {
    /* Verify that the magnetized angle is between the input angle and the angle between robot and goal. */
    ChassisSpeeds robotSpeed = new ChassisSpeeds(2.0, 3.0, 0.0);
    Pose2d robotPose = new Pose2d(2.0, 2.0, new Rotation2d());
    Pose2d goalPose = new Pose2d(2.0, 3.0, new Rotation2d());
    Rotation2d robotToGoal = Rotation2d.fromDegrees(90);

    ChassisSpeeds magnetizedSpeed =
        MagnetismUtil.getMagnetizedChassisSpeeds(robotSpeed, robotPose, goalPose);

    Assertions.assertTrue(
        robotToGoal.getDegrees() > getAngleFromChassisSpeeds(magnetizedSpeed).getDegrees());
    Assertions.assertTrue(
        getAngleFromChassisSpeeds(magnetizedSpeed).getDegrees()
            > getAngleFromChassisSpeeds(robotSpeed).getDegrees());
  }

  @Test
  void testMagnetismAngleGetsStrongerCloserToGoal() {
    /* Test angle when robot is closer to goal is stronger than when farther. */
    ChassisSpeeds robotSpeed = new ChassisSpeeds(2.0, 3.0, 0.0);
    Pose2d robotPoseFar = new Pose2d(2.0, 2.0, new Rotation2d());
    Pose2d robotPoseClose = new Pose2d(2.0, 2.5, new Rotation2d());
    Pose2d goalPose = new Pose2d(2.0, 3.0, new Rotation2d());

    ChassisSpeeds magnetizedSpeedClose =
        MagnetismUtil.getMagnetizedChassisSpeeds(robotSpeed, robotPoseClose, goalPose);
    ChassisSpeeds magnetizedSpeedFar =
        MagnetismUtil.getMagnetizedChassisSpeeds(robotSpeed, robotPoseFar, goalPose);

    Assertions.assertTrue(
        getAngleFromChassisSpeeds(magnetizedSpeedClose).getDegrees()
            > getAngleFromChassisSpeeds(magnetizedSpeedFar).getDegrees());
  }
}
