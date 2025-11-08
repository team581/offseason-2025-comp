package frc.robot.climber;

public enum ClimberState {
  STOPPED(100.0, 0.0),

  LINEUP_FORWARD(0, 2.5),
  LINEUP_BACKWARD(30, 2.5),
  HANGING(185.0, 12.0);

  public final double angle;
  public final double forwardsVoltage;

  private ClimberState(double angle, double forwardVoltage) {
    this.angle = angle;
    this.forwardsVoltage = forwardVoltage;
  }
}
