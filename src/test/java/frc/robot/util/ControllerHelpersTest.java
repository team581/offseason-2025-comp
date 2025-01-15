package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

public class ControllerHelpersTest {
  private static Translation2d round(Translation2d input) {
    return new Translation2d(
        MathHelpers.roundTo(input.getX(), 0.01), MathHelpers.roundTo(input.getY(), 0.01));
  }

  @Test
  void circularDiscCoordinateTest() {
    var result = ControllerHelpers.fromCircularDiscCoordinates(-0.68, -0.58);
    Translation2d expected = new Translation2d(-0.78, -0.7);

    assertEquals(expected, round(result));
  }
}
