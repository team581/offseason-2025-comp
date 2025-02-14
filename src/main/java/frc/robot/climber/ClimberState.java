package frc.robot.climber;

public enum ClimberState {
  STOWED(167.0),
  LINEUP(-20),
  HANGING(120.0);

  public final double angle;

  private ClimberState(double angle) {
    this.angle = angle;
  }
}
