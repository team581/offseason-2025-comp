package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.MathHelpers;
import frc.robot.vision.game_piece_detection.GamePieceDetectionUtils;
import org.junit.jupiter.api.Test;

public class GamePieceDetectionTest {
  @Test
  public void gamePieceDetectionTest() {
    var result =
        GamePieceDetectionUtils.calculateFieldRelativeTranslationFromCamera(
            0, 0, new Pose2d(), new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
    assertEquals(new Translation2d(0, 0), result);
  }

  @Test
  public void gamePieceDetectionTestTY() {
    var result =
        GamePieceDetectionUtils.calculateFieldRelativeTranslationFromCamera(
            0,
            11.31,
            new Pose2d(),
            new Pose3d(0, 0, 5, new Rotation3d(0, Units.degreesToRadians(180 + 90), 0)));
    assertEquals(
        MathHelpers.roundTo(new Translation2d(1.0, 0.0), 0.01), MathHelpers.roundTo(result, 0.01));
  }
}
