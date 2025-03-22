package frc.robot.climber;

public enum ClimberState {
  STOWED(98.0, -2),
  LINEUP(15.0, -12),
  HANGING(99.0, -12);

  public final double angle;
  public final double forwardsVoltage;

  private ClimberState(double angle, double forwardVoltage) {
    this.angle = angle;
    this.forwardsVoltage = forwardVoltage;
  }
}
