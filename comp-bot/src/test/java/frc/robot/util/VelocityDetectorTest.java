package frc.robot.util;

import static org.assertj.core.api.Assertions.assertThat;

import com.team581.mechanisms.VelocityDetector;
import org.junit.jupiter.api.Test;

final class VelocityDetectorTest {
  @Test
  void detectionSequence() {
    var detector = new VelocityDetector(10, Double.MAX_VALUE, 0.0);

    // Velocity is below the min threshold
    assertThat(detector.hasGamePiece(0, 5)).isFalse();
    // Velocity is above the min threshold
    assertThat(detector.hasGamePiece(15, 5)).isFalse();
    // Velocity has reached the min, and now is below max
    assertThat(detector.hasGamePiece(3, 5)).isTrue();
    detector.reset();
    // Velocity is below max, but detector was reset
    assertThat(detector.hasGamePiece(3, 5)).isFalse();
  }
}
