package frc.robot.singulator;

public enum SingulatorState {
  UNTUNED(0.0),

  UNJAM_RIGHT_ONLY(UNTUNED),
  UNJAM_LEFT_ONLY(UNTUNED),

  STOPPED(0.0),
  IDLE(UNTUNED),
  INTAKING(6),
  OUTTAKING(UNTUNED),
  L1_SCORE(UNTUNED),
  HANDOFF(UNTUNED);

  public final double voltsLeft;
  public final double voltsRight;

  private SingulatorState(SingulatorState state) {
    this.voltsLeft = state.voltsLeft;
    this.voltsRight = state.voltsRight;
  }

  private SingulatorState(double voltsBoth) {
    this.voltsLeft = voltsBoth;
    this.voltsRight = voltsBoth;
  }

  private SingulatorState(double voltsLeft, double voltsRight) {
    this.voltsLeft = voltsLeft;
    this.voltsRight = voltsRight;
  }
}
