package frc.robot.auto_align;

public enum AutoAlignState {
  // TODO: Ponder having teleop & auto states
  SAFE_WAITING,
  SAFE_WAITING_LEFT_CENTER,
  SAFE_WAITING_RIGHT_CENTER,
  SAFE_WAITING_LEFT_RAISING,
  SAFE_WAITING_RIGHT_RAISING,
  SAFE_PREPARE,
  LEFT_PIPE,
  RIGHT_PIPE,
  ALGAE;
}
