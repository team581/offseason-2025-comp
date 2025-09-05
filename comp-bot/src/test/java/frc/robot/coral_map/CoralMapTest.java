package frc.robot.coral_map;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.team581.util.FmsUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.auto_align.AutoAlign;
import frc.robot.vision.game_piece_detection.CoralMap;
import org.junit.jupiter.api.Test;

public class CoralMapTest {
  @Test
  void isCoralSafeForAuto() {
    var centerOfReef = AutoAlign.getAllianceCenterOfReef(FmsUtil.isRedAlliance());
    var coralTranslation = new Translation2d(centerOfReef.getX() - 0.5, centerOfReef.getY());
    var result = CoralMap.isCoralInSafeSpotForAuto(coralTranslation);
    var expected = false;
    assertEquals(result, expected);
  }
}
