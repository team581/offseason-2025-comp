package frc.robot.elevator;

public enum CoralStation {
  PROCESSOR_SIDE_RED(37.75),
  PROCESSOR_SIDE_BLUE(37.25),
  NON_PROCESSOR_SIDE_RED(37.25),
  NON_PROCESSOR_SIDE_BLUE(37.25);

  /** The height in inches from the carpet to the bottom of the coral station plastic. */
  private static final double IDEAL_HEIGHT = 37.25;

  /** The number of inches to add to the elevator height to intake from this station. */
  public final double offset;

  CoralStation(double measuredHeight) {
    offset = IDEAL_HEIGHT - measuredHeight;
  }
}
