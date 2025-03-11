package frc.robot.game_piece_detection;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.vision.game_piece_detection.GamePieceDetectionUtil;
import frc.robot.vision.results.GamePieceResult;

public class GamePieceDetectionUtilsTest {
  @Test
  public void testCalculateFieldRelativeTranslationFromCamera() {
    var tx = -2.63;
    var ty = -16.4;
    var expected = Units.inchesToMeters(34);
    var actual = GamePieceDetectionUtil.calculateRobotRelativeTranslationFromCamera(new GamePieceResult(tx, ty, 0.0), GamePieceDetectionUtil.LIMELIGHT_POSE_TO_ROBOT).getX();
    assertTrue(MathUtil.isNear(expected, actual, Units.inchesToMeters(2.0)));
  }
}
