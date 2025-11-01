package frc.robot.climber;

public enum ClimberState {
  STOWED(98.0, 0.0),
  LINEUP(0, 6.0),
  HANGING(99.0, 12.0);

  public final double angle;
  public final double voltage;

  private ClimberState(double angle, double voltage) {
    this.angle = angle;
    this.voltage = voltage;
  }
}
