package frc.robot.auto_align;

public enum ReefAlignState {
  ALL_CAMERAS_DEAD,

  // Checking based on tags
  NO_TAGS_WRONG_POSITION,
  NO_TAGS_IN_POSITION,
  HAS_TAGS_WRONG_POSITION,
  HAS_TAGS_IN_POSITION;
}
