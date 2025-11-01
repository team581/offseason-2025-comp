package frc.robot.climber;

public enum ClimberState {
  STOWED(98.0, 0.0, 0.0, 0.0),
  LINEUP(0, 6.0, 0.0, -6.0),
  HANGING(99.0, 12.0, -1.0, -12.0);

  public final double angle;
  public final double forwardsVoltage;
  public final double backwardsVoltage;
  public final double holdingVoltage;

  private ClimberState(
      double angle, double forwardsVoltage, double holdingVoltage, double backwardsVoltage) {
    this.angle = angle;
    this.forwardsVoltage = forwardsVoltage;
    this.holdingVoltage = holdingVoltage;
    this.backwardsVoltage = backwardsVoltage;
  }
}
