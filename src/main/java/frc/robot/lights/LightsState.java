package frc.robot.lights;

import edu.wpi.first.wpilibj.util.Color;

public enum LightsState {
  CAMERA_DEAD(Color.kRed, BlinkPattern.BLINK_FAST),

  IDLE_NO_GP_CORAL_MODE(Color.kWhite, BlinkPattern.BLINK_SLOW),
  IDLE_WITH_CORAL(Color.kWhite, BlinkPattern.SOLID),

  IDLE_NO_GP_ALGAE_MODE(Color.kTurquoise, BlinkPattern.BLINK_SLOW),
  IDLE_WITH_ALGAE(Color.kTurquoise, BlinkPattern.SOLID),

  APPROACHING_REEF(Color.kGold, BlinkPattern.SOLID),
  // TODO: Implement code that dynamically checks if you are actually aligned or not
  CORAL_SCORE_ALIGNMENT(Color.kLimeGreen, BlinkPattern.SOLID),
  NET_SCORE_ALIGNMENT(Color.kLimeGreen, BlinkPattern.SOLID),
  PROCESSOR_SCORE_ALIGNMENT(Color.kLimeGreen, BlinkPattern.SOLID),

  SCORING(Color.kLimeGreen, BlinkPattern.BLINK_FAST),

  PLACEHOLDER(Color.kBlack, BlinkPattern.SOLID);

  public final BlinkPattern pattern;
  public final Color color;

  LightsState(Color color, BlinkPattern pattern) {
    this.pattern = pattern;
    this.color = color;
  }
}
