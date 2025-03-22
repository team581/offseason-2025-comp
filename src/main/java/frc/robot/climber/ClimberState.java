package frc.robot.climber;

public enum ClimberState {
  STOWED(98.0, -0),
  LINEUP(15.0, -0),
  HANGING(99.0, -0);

  public final double angle;
  public final double forwardsVoltage;

  private ClimberState(double angle, double forwardVoltage) {
    this.angle = angle;
    this.forwardsVoltage = forwardVoltage;
  }
}
