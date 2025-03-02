package frc.robot.climber;

public enum ClimberState {
  STOWED(195, 2, -2),
  LINEUP(0, 12, -12),
  HANGING(156.0, 12, -12),
  HANGING_2(160.0, 12, -12),
  HANGING_3(165.0, 12, -12);

  public final double angle;
  public final double forwardsVoltage;
  public final double backwardsVoltage;

  private ClimberState(double angle, double forwardVoltage, double backwardsVoltage) {
    this.angle = angle;
    this.forwardsVoltage = forwardVoltage;
    this.backwardsVoltage = backwardsVoltage;
  }
}
