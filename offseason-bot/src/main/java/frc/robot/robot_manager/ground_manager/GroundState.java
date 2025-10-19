package frc.robot.robot_manager.ground_manager;

public enum GroundState {
  DEPLOY_NOT_HOMED,
  DEPLOY_HOMING,
  UNJAM_LEFT,
  UNJAM_RIGHT,
  IDLE_GP,
  IDLE_NO_GP,
  INTAKING,
  OUTTAKING,
  HANDOFF_WAIT,
  HANDOFF_RELEASE,
  INTAKE_THEN_HANDOFF_WAIT,
  CLIMB;
}
