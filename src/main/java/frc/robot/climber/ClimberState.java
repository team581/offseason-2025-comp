package frc.robot.climber;

public enum ClimberState {
  STOWED(167.0),
  // Need to be very careful that this number is reachable without hitting the bumpers (lower
  // hardstop). Otherwise the climber will do evil things.
  LINEUP(-15),
  HANGING(115.0);

  public final double angle;

  private ClimberState(double angle) {
    this.angle = angle;
  }
}
