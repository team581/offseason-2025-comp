package frc.robot.climber;

public enum ClimberState {
  STOWED(98.0, -2, 2),
  LINEUP(0.0, -12, 12),
  HANGING(90.0, -12, 12),
  HANGING_2(92.0, -12, 12),
  HANGING_3(94.0, -12, 12);

  public final double angle;
  public final double forwardsVoltage;
  public final double backwardsVoltage;

  private ClimberState(double angle, double forwardVoltage, double backwardsVoltage) {
    this.angle = angle;
    this.forwardsVoltage = forwardVoltage;
    this.backwardsVoltage = backwardsVoltage;
  }
}
