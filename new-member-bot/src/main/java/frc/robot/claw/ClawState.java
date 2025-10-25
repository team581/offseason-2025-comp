package frc.robot.claw;

public enum ClawState {
  UNTUNED(0.0),

  IDLE_NO_GP(UNTUNED),
  IDLE_W_ALGAE(UNTUNED),
  IDLE_W_CORAL(UNTUNED),
  INTAKING_CORAL(UNTUNED),
  INTAKING_ALGAE(UNTUNED),
  SCORE_ALGAE_NET(UNTUNED),
  SCORE_ALGAE_PROCESSOR(UNTUNED),
  SCORE_CORAL(UNTUNED),
  OUTTAKING(UNTUNED);

  public final double volts;

  private ClawState(double volts) {
    this.volts = volts;
  }

  private ClawState(ClawState state) {
    this.volts = state.volts;
  }
}
