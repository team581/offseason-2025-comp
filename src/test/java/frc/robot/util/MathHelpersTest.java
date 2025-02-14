package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class MathHelpersTest {
  @Test
  void roundToTest() {
    var result = MathHelpers.roundTo(123.45, 0.1);
    var expected = 123.5;

    assertEquals(expected, result);
  }

  @Test
  void signedExpTest() {
    var result = MathHelpers.signedExp(-4, 2);

    assertEquals(-16, result);
  }

  @Test
  void signedSqrtTest() {
    var result = MathHelpers.signedSqrt(-36);

    assertEquals(-6, result);
  }
}
