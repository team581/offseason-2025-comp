package frc.robot.auto_align;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class AutoAlignTest {

  @Test
  void scoreInNetOnBlueSideAndLeft() {
    // x range: 0-8.775 for blue, 8.775-17.55 for red
    var robotPose = new Pose2d(7, 0, Rotation2d.fromDegrees(218));

    var result = AutoAlign.getNetScoringSideFromRobotPose(robotPose);

    Assertions.assertEquals(RobotScoringSide.LEFT, result);
  }

  @Test
  void scoreInNetOnBlueSideAndRight() {
    // x range: 0-8.775 for blue, 8.775-17.55 for red
    var robotPose = new Pose2d(3, 0, Rotation2d.fromDegrees(23));

    var result = AutoAlign.getNetScoringSideFromRobotPose(robotPose);

    Assertions.assertEquals(RobotScoringSide.RIGHT, result);
  }

  @Test
  void scoreInNetOnRedSideAndLeft() {
    // x range: 0-8.775 for blue, 8.775-17.55 for red
    var robotPose = new Pose2d(9, 0, Rotation2d.fromDegrees(67));

    var result = AutoAlign.getNetScoringSideFromRobotPose(robotPose);

    Assertions.assertEquals(RobotScoringSide.LEFT, result);
  }

  @Test
  void scoreInNetOnRedSideAndRight() {
    // x range: 0-8.775 for blue, 8.775-17.55 for red
    var robotPose = new Pose2d(13, 0, Rotation2d.fromDegrees(224));

    var result = AutoAlign.getNetScoringSideFromRobotPose(robotPose);

    Assertions.assertEquals(RobotScoringSide.RIGHT, result);
  }

  @Test
  void scoreRightOrLeft() {
    var robotPose = new Pose2d(13, 0, Rotation2d.fromDegrees(180.0));
    var result = AutoAlign.getScoringSideFromRobotPose(robotPose, true, true);
    assertEquals(RobotScoringSide.RIGHT, result);
  }
}
