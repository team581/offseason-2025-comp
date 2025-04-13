package frc.robot.vision.results;

import frc.robot.util.ReusableOptional;

public class OptionalGamePieceResult extends ReusableOptional<GamePieceResult> {
  public OptionalGamePieceResult() {
    super(new GamePieceResult());
  }

  public OptionalGamePieceResult update(double tx, double ty, double timestamp) {
    this.value.update(tx, ty, timestamp);
    this.isPresent = true;

    return this;
  }

  public OptionalGamePieceResult empty() {
    this.value.update(0.0, 0.0, 0.0);
    this.isPresent = false;
    return this;
  }
}
