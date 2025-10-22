package frc.robot.elevator;

public enum ElevatorState {
  UNTUNED(0.0),
  UNJAM(UNTUNED),
  REHOME(UNTUNED),

  ALGAE_INTAKE_GROUND(UNTUNED),
  ALGAE_INTAKE_L2(UNTUNED),
  ALGAE_INTAKE_L3(UNTUNED),

  ALGAE_NET(50),
  PROCESSOR(UNTUNED),
  ALGAE_OUTTAKE(UNTUNED),

  STOWED(5),
  STOWED_INWARD(UNTUNED),
  PRE_CORAL_HANDOFF(5),
  HANDOFF(0),

  L1_SCORE_LINEUP(10),
  L1_SCORE_RELEASE(10),

  L2_SCORE_LINEUP(20),
  L2_SCORE_RELEASE(20),

  L3_SCORE_LINEUP(30),
  L3_SCORE_RELEASE(30),

  L4_SCORE_LINEUP(40),
  L4_SCORE_RELEASE(40),

  CLIMBING(UNTUNED);

  public final double height;

  public double getHeight() {
    return this.height;
  }

  private ElevatorState(double height) {
    this.height = height;
  }

  private ElevatorState(ElevatorState state) {
    this.height = state.height;
  }
}
