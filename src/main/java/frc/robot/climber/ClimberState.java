package frc.robot.climber;

public enum ClimberState {
  STOWED(0.0),
  LINEUP(0.0),
  HANGING(0.0);

  public final double angle;

  private ClimberState(double angle) {
    this.angle = angle;
  }
}
