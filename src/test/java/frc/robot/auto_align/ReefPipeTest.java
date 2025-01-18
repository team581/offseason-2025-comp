package frc.robot.auto_align;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.MathHelpers;
import org.junit.jupiter.api.Test;

public class ReefPipeTest {
  private static final Translation2d BLUE_REEF_CENTER = new Translation2d(4.4789, 4.0259);
  private static final Translation2d RED_REEF_CENTER = new Translation2d(13.0447, 4.0259);

  private static final double BLUE_PIPE_A_TO_BLUE_ALLIANCE_WALL_DISTANCE = 3.6982;
  private static final double RED_PIPE_A_TO_BLUE_ALLIANCE_WALL_DISTANCE = 13.8254;

  private static double round(double value) {
    // Round to centimeter precision
    return MathHelpers.roundTo(value, 0.01);
  }

  @Test
  void checkBlueReefPipeDistancesToCenter() {
    var wantedDistance =
        round(ReefPipe.PIPE_A.bluePose.getTranslation().getDistance(BLUE_REEF_CENTER));

    for (var pipe : ReefPipe.values()) {
      var distance = pipe.bluePose.getTranslation().getDistance(BLUE_REEF_CENTER);
      assertEquals(wantedDistance, round(distance));
    }
  }

  @Test
  void checkRedReefPipeDistancesToCenter() {
    var wantedDistance =
        round(ReefPipe.PIPE_A.redPose.getTranslation().getDistance(RED_REEF_CENTER));

    for (var pipe : ReefPipe.values()) {
      var distance = pipe.redPose.getTranslation().getDistance(RED_REEF_CENTER);
      assertEquals(wantedDistance, round(distance));
    }
  }

  @Test
  void checkBlueReefPipeADistanceToWall() {
    assertEquals(
        round(BLUE_PIPE_A_TO_BLUE_ALLIANCE_WALL_DISTANCE), round(ReefPipe.PIPE_A.bluePose.getX()));
  }

  @Test
  void checkRedReefPipeADistanceToWall() {
    assertEquals(
        round(RED_PIPE_A_TO_BLUE_ALLIANCE_WALL_DISTANCE), round(ReefPipe.PIPE_A.redPose.getX()));
  }
}
