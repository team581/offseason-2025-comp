package frc.robot.climber;

public enum ClimberState {
  STOWED(167.0, 1, -1),
  // Need to be very careful that this number is reachable without hitting the bumpers (lower
  // hardstop). Otherwise the climber will do evil things.
  LINEUP(-15, 8, -8),
  HANGING(115.0, 12, -12);

  public final double angle;
  public final double forwardsVoltage;
  public final double backwardsVoltage;

  private ClimberState(double angle, double forwardVoltage, double backwardsVoltage) {
    this.angle = angle;
    this.forwardsVoltage = forwardVoltage;
    this.backwardsVoltage = backwardsVoltage;
  }
}
