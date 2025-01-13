package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class AutoAlignTest {
  @Test
  void scoreInNetOnBlueSideAndFacingNet() {
    // x range: 0-8.775 for blue, 8.775-17.55 for red
    var robotPose = new Pose2d(7, 0, Rotation2d.fromDegrees(23));

    var result = AutoAlign.shouldNetScoreForwards(robotPose);

    Assertions.assertEquals(true, result);
  }

  @Test
  void scoreInNetOnBlueSideAndFacingAway() {
    // x range: 0-8.775 for blue, 8.775-17.55 for red
    var robotPose = new Pose2d(3, 0, Rotation2d.fromDegrees(218));

    var result = AutoAlign.shouldNetScoreForwards(robotPose);

    Assertions.assertEquals(false, result);
  }

  @Test
  void scoreInNetOnRedSideAndFacingNet() {
    // x range: 0-8.775 for blue, 8.775-17.55 for red
    var robotPose = new Pose2d(9, 0, Rotation2d.fromDegrees(185));

    var result = AutoAlign.shouldNetScoreForwards(robotPose);

    Assertions.assertEquals(true, result);
  }

  @Test
  void scoreInNetOnRedSideAndFacingAway() {
    // x range: 0-8.775 for blue, 8.775-17.55 for red
    var robotPose = new Pose2d(13, 0, Rotation2d.fromDegrees(67));

    var result = AutoAlign.shouldNetScoreForwards(robotPose);

    Assertions.assertEquals(false, result);
  }
}
