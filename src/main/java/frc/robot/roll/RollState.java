package frc.robot.roll;

public enum RollState {
  UNHOMED(0.0),
  HOMING(0.0),
  CORAL_HORIZONTAL(0.0),
  CORAL_UPRIGHT(90),
  ALGAE(0.0),
  FLOOR_ALGAE(0.0),
  SMART_STOW(0.0),
  CORAL_SCORE(0.0),
  CORAL_SCORE_L1(0.0),
  DEMO_1(-90),
  DEMO_2(90);

  public final double angle;

  RollState(double angle) {
    this.angle = angle;
  }
}
