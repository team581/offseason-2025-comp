package frc.robot.climber;

public enum ClimberState {
  STOWED(-90.0, 2.0, 0.0, -2.0),
  LINEUP(0, 6.0, 0.0, -6.0),
  HANGING(-108.0, 12.0, 0.0, -12.0);

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
