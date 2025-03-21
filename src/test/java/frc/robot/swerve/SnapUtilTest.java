package frc.robot.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

public class SnapUtilTest {
  @Test
  void getForwardSnapAngleBlueSide() {
    var robotPose = new Pose2d(0, 0, Rotation2d.kZero);
    var expected = 0.0;
    var result = SnapUtil.getLeftNetDirection(robotPose);
    assertEquals(expected, result);
  }

  @Test
  void getForwardSnapAngleRedSide() {
    var robotPose = new Pose2d(15, 0, Rotation2d.kZero);
    var expected = 180.0;
    var result = SnapUtil.getLeftNetDirection(robotPose);
    assertEquals(expected, result);
  }

  @Test
  void getBackwardSnapAngleBlueSide() {
    var robotPose = new Pose2d(0, 0, Rotation2d.kZero);
    var expected = 0.0;
    var result = SnapUtil.getLeftNetDirection(robotPose);
    assertEquals(expected, result);
  }

  @Test
  void getBackwardSnapAngleRedSide() {
    var robotPose = new Pose2d(15, 0, Rotation2d.kZero);
    var expected = 180.0;
    var result = SnapUtil.getLeftNetDirection(robotPose);
    assertEquals(expected, result);
  }
}
