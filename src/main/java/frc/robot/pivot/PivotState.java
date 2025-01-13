package frc.robot.pivot;

public enum PivotState {
  HOMING(0.0),
  INTAKING_CORAL_HORIZONTAL(0.0),
  STOWED(0.0),
  CORAL_SCORE(0.0);

  public final double angle;

  PivotState(double angle) {
    this.angle = angle;
  }
}
