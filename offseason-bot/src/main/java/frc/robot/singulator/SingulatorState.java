package frc.robot.singulator;

public enum SingulatorState {
  UNTUNED(0.0),

  UNJAM_RIGHT_ONLY(UNTUNED),
  UNJAM_LEFT_ONLY(UNTUNED),

  STOPPED(0.0),
  IDLE(0.0),
  INTAKING(6),
  OUTTAKING(-1.0),
  HANDOFF(0.0);

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
}
