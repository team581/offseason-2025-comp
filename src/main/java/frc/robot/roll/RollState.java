package frc.robot.roll;

public enum RollState {
  UNHOMED(0.0),
  HOMING(0.0),
  INTAKING_CORAL_HORIZONTAL(0.0),
  STOWED(0.0),
  CORAL_SCORE(0.0);

  public final double angle;

  RollState(double angle) {
    this.angle = angle;
  }
}
