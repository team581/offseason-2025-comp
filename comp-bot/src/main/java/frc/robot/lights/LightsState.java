package frc.robot.lights;

public enum LightsState {
  ERROR,
  UNHOMED,
  HOMED_NO_TAGS,
  HOMED_SEES_TAGS,

  BLINK,

  INTAKING_CORAL,
  INTAKING_ALGAE,

  IDLE_EMPTY,
  HOLDING_CORAL,
  HOLDING_ALGAE,

  LOLLIPOP_SEES_ALGAE,
  LOLLIPOP_NO_ALGAE,

  CORAL_HANDOFF,

  CLIMB_LINEUP,
  CLIMB_HANG,
  CLIMB_STOP,

  SCORE_NO_ALIGN_NO_TAGS,
  SCORE_NO_ALIGN_TAGS,
  SCORE_ALIGN_NO_TAGS,
  SCORE_ALIGN_TAGS,

  SCORING_ALGAE,
  SCORING_CORAL,

  OTHER,

  /**
   * @deprecated Replace placeholder lights with actual light patterns.
   */
  @Deprecated
  PLACEHOLDER;
}
