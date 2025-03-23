package frc.robot.intake_deploy;

public enum DeployState {
  UNHOMED(0.0),
  HOMING(0.0), // TODO: set homing end angle
  STOWED(118),
  FLOOR_INTAKE(-34),
  HANDOFF(118),
  L1_SCORE(80),

  UNJAM(-28);

  public final double angle;

  private DeployState(double angle) {
    this.angle = angle;
  }
}
