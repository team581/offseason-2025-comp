package frc.robot.lights;

import edu.wpi.first.wpilibj.util.Color;

public enum LightsState {
  ERROR(Color.kRed, BlinkPattern.BLINK_FAST),
  UNHOMED(Color.kYellow, BlinkPattern.BLINK_SLOW),
  HEALTHY(Color.kGreen, BlinkPattern.SOLID),

  IDLE_NO_GP_CORAL_MODE(Color.kWhite, BlinkPattern.BLINK_SLOW),
  IDLE_WITH_CORAL(Color.kWhite, BlinkPattern.SOLID),

  IDLE_NO_GP_ALGAE_MODE(Color.kTeal, BlinkPattern.BLINK_SLOW),
  IDLE_WITH_ALGAE(Color.kTeal, BlinkPattern.SOLID),

  SCORE_ALIGN_NOT_READY(Color.kYellow, BlinkPattern.SOLID),
  SCORE_ALIGN_READY(Color.kGreen, BlinkPattern.SOLID),

  SCORING(Color.kGreen, BlinkPattern.BLINK_FAST),

  PLACEHOLDER(Color.kBlack, BlinkPattern.SOLID);

  public final BlinkPattern pattern;
  public final Color color;

  LightsState(Color color, BlinkPattern pattern) {
    this.pattern = pattern;
    this.color = color;
  }
}
