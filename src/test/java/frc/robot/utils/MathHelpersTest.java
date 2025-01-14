package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.util.MathHelpers;
import org.junit.jupiter.api.Test;

public class MathHelpersTest {
  @Test
  void roundToTest() {
    var result = MathHelpers.roundTo(123.45, 0.1);
    var expected = 123.5;

    assertEquals(expected, result);
  }
}
