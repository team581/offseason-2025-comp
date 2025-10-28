package frc.robot.singulator;

public enum SingulatorState {
  UNTUNED(0.0),

  UNJAM_RIGHT_ONLY(6.0, -6.0),
  UNJAM_LEFT_ONLY(-6.0, 6.0),

  STOPPED(0.0),
  IDLE(0.0),
  INTAKING(6.0),
  OUTTAKING(-6.0),
  HANDOFF(0.0);

  public final double voltsLeft;
  public final double voltsRight;

  private SingulatorState(double voltsBoth) {
    this.voltsLeft = voltsBoth;
    this.voltsRight = voltsBoth;
  }

  private SingulatorState(double voltsLeft, double voltsRight) {
    this.voltsLeft = voltsLeft;
    this.voltsRight = voltsRight;
  }
}
