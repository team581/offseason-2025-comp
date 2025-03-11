package frc.robot.vision.game_piece_detection;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.vision.results.GamePieceResult;
import org.junit.jupiter.api.Test;

public class GamePieceDetectionUtilTest {
  @Test
  public void lollipopStraightOnPoseTest() {
    var result =
        GamePieceDetectionUtil.calculateFieldRelativeLollipopTranslationFromCamera(
            new Pose2d(15.06, 3.81, Rotation2d.fromDegrees(-3.0)),
            new GamePieceResult(-13.28, 10.37, 0)); // : -13.28° ty: +10.37°
    var expected = new Translation2d(16.33, 4.02);
    assertEquals(expected, result);
  }

  @Test
  public void lollipopFromSidePoseTest() {
    var result =
        GamePieceDetectionUtil.calculateFieldRelativeLollipopTranslationFromCamera(
            new Pose2d(15.18, 2.85, Rotation2d.fromDegrees(35.58)),
            new GamePieceResult(-13.28, 10.37, 0)); // :tx: -14.57 ty: +14.25°

    var expected = new Translation2d(16.33, 4.02);
    assertEquals(expected, result);
  }
}
