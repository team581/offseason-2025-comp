package frc.robot.claw;

public enum ClawState {
  UNTUNED(0.0),

  IDLE_NO_GP(0.0),
  IDLE_W_ALGAE(-10.0),
  IDLE_W_CORAL(0.3),
  INTAKING_CORAL(12.0),
  INTAKING_ALGAE(-12.0),
  SCORE_ALGAE_NET(UNTUNED),
  SCORE_ALGAE_PROCESSOR(UNTUNED),
  SCORE_CORAL(-1.5),
  // TODO: SEPARATE OUTTAKING FOR CORAL AND ALGAE
  OUTTAKING(-1.5);

  public final double volts;

  private ClawState(double volts) {
    this.volts = volts;
  }

  private ClawState(ClawState state) {
    this.volts = state.volts;
  }
}
