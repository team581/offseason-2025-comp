package frc.robot.lights;

import edu.wpi.first.wpilibj.util.Color;

public enum LightsState {
  // TODO: Work with Saikiran to finalize these values
  ERROR(Color.kRed, BlinkPattern.BLINK_FAST),
  READY(Color.kGreen, BlinkPattern.BLINK_FAST),
  IN_PROGRESS(Color.kYellow, BlinkPattern.BLINK_SLOW),

  PLACEHOLDER(Color.kBlack, BlinkPattern.SOLID);

  public final BlinkPattern pattern;
  public final Color color;

  LightsState(Color color, BlinkPattern pattern) {
    this.pattern = pattern;
    this.color = color;
  }
}
