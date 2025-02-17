package frc.robot.climber;

public enum ClimberState {
  STOWED(167.0, 2, -2),
  LINEUP(-15, 12, -12),
  HANGING(115.0, 12, -12),
  HANGING_2(113.0, 12, -12),
  HANGING_3(111.0, 12, -12);

  public final double angle;
  public final double forwardsVoltage;
  public final double backwardsVoltage;

  private ClimberState(double angle, double forwardVoltage, double backwardsVoltage) {
    this.angle = angle;
    this.forwardsVoltage = forwardVoltage;
    this.backwardsVoltage = backwardsVoltage;
  }
}
