package frc.robot.wrist;

public enum WristState {
  IDLE(0.0),
  UNJAM(0.0),
  PRE_MATCH_HOMING(0.0),

  CORAL_SCORE_LV1(0.0),
  CORAL_SCORE_LV2(0.0),
  CORAL_SCORE_LV3(0.0),
  CORAL_SCORE_LV4(0.0),

  ALGAE_FORWARD_NET(0.0),
  ALGAE_BACKWARD_NET(0.0),
  ALGAE_PROCESSOR(0.0),

  SOURCE_INTAKE(0.0),
  GROUND_CORAL_INTAKE(0.0),
  GROUND_ALGAE_INTAKE(0.0);

  public final double angle;

  WristState(double angle) {
    this.angle = angle;
  }
}
