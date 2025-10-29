package frc.robot.intake;

public enum IntakeState {
  UNTUNED(0.0),
  UNJAM(UNTUNED),

  STOPPED(0.0),
  IDLE(0.0),
  INTAKING(12),
  OUTTAKING(-12),
  HANDOFF(0.0);

  public final double volts;

  private IntakeState(IntakeState state) {
    this.volts = state.volts;
  }

  private IntakeState(double volts) {
    this.volts = volts;
  }
}
