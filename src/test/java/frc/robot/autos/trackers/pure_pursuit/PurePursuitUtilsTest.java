package frc.robot.autos.trailblazer.trackers.pure_pursuit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autos.trackers.pure_pursuit.PurePursuitUtils;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class PurePursuitUtilsTest {
  @Test
  void getPerpindicularWaypoint() {
    var startPoint = new Pose2d(-1, -4, new Rotation2d());
    var endPoint = new Pose2d(5, 2, new Rotation2d());
    var robotPose = new Pose2d(-4.0, -4.0, new Rotation2d());
    var result = PurePursuitUtils.getPerpendicularPoint(startPoint, endPoint, robotPose);

    var expected = new Pose2d(-2.5, -5.5, new Rotation2d());
    Assertions.assertEquals(expected, result);
  }

  @Test
  void getLookaheadWaypoint() {
    var startPoint = new Pose2d(0, 0, new Rotation2d());
    var endPoint = new Pose2d(5, 0, new Rotation2d());
    var pointOnPath = new Pose2d(2, 0, new Rotation2d());
    var result = PurePursuitUtils.getLookaheadPoint(startPoint, endPoint, pointOnPath, 1.0);

    var expected = new Pose2d(3, 0, new Rotation2d());
    Assertions.assertEquals(expected, result);
  }
}
