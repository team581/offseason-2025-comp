package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class VelocityDetectorTest {
  @Test
  void testDetectionSequence() {
    var detector = new VelocityDetector(0.8, 1.0);
    // motor velocity is under the minimum intaking threshold
    var velocityUnderMinThreshold = 0.7;
    // motor velocity has crossed the minimum intaking threshold, now waiting to go below max
    // threshold
    var velocityAboveMinThreshold = 1.1;
    // motor velocity under max threshold while holding gamepiece, return true
    var velocityUnderMaxThreshold = 0.7;
    // motor velocity above max threshold while holding gamepiece, return false
    var velocityAboveMaxThreshold = 1.1;
    assertFalse(detector.hasGamePiece(velocityUnderMinThreshold));
    assertFalse(detector.hasGamePiece(velocityAboveMinThreshold));
    assertTrue(detector.hasGamePiece(velocityUnderMaxThreshold));
    assertFalse(detector.hasGamePiece(velocityAboveMaxThreshold));
  }
}
