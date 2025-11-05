package frc.robot.climber;

public enum ClimberState {
  STOPPED(100.0, 0.0),

  LINEUP_FORWARD(0, 3.0),
  LINEUP_BACKWARD(0, 3.0),
  HANGING(170.0, 12.0);

  public final double angle;
  public final double forwardsVoltage;

  private ClimberState(double angle, double forwardVoltage) {
    this.angle = angle;
    this.forwardsVoltage = forwardVoltage;
  }
}
