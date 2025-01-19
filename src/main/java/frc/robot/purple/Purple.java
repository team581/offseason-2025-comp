package frc.robot.purple;

import edu.wpi.first.math.MathUtil;
import frc.robot.vision.limelight.Limelight;

public class Purple {
  private final Limelight camera;

  public Purple(Limelight camera) {
    this.camera = camera;
  }

  public PurpleState getPurpleState() {
    var maybeResult = camera.getPurpleResult();
    if (maybeResult.isEmpty()) {
      return PurpleState.NO_PURPLE;
    }

    var result = maybeResult.get();

    if (MathUtil.isNear(0, result.tx(), 1)) {
      return PurpleState.CENTERED;
    }

    return PurpleState.VISIBLE_NOT_CENTERED;
  }
}
