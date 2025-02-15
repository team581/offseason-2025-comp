package frc.robot.auto_align;

public enum ReefAlignState {
  TAG_CAMERAS_DEAD,
  PURPLE_CAMERA_DEAD,
  ALL_CAMERAS_DEAD,

  // Checking based on tags
  NO_TAGS_WRONG_POSITION,
  NO_TAGS_IN_POSITION,
  HAS_TAGS_WRONG_POSITION,
  HAS_TAGS_IN_POSITION,

  // Checking based on purple
  HAS_PURPLE_NOT_ALIGNED,
  HAS_PURPLE_ALIGNED;
}
