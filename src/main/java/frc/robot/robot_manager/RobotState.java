package frc.robot.robot_manager;

public enum RobotState {
  // Idle states
  /** Idle without any game piece. */
  CLAW_EMPTY_DEPLOY_EMPTY,
  /** Deploy is holding coral, claw isn't doing anything. */
  CLAW_EMPTY_DEPLOY_CORAL,
  /** Claw holding algae, deploy is holding coral. */
  CLAW_ALGAE_DEPLOY_CORAL,
  /** Claw holding algae, deploy is empty. */
  CLAW_ALGAE_DEPLOY_EMPTY,
  /** Claw holding coral, deploy is empty. */
  CLAW_CORAL_DEPLOY_EMPTY,

  // Intake and outtake states
  CORAL_INTAKE_FLOOR_CLAW_EMPTY,
  CORAL_INTAKE_FLOOR_CLAW_ALGAE,
  // In theory we could have intake upright while holding algae but nobody is going to use that
  CORAL_INTAKE_UPRIGHT_CLAW_EMPTY,
  // Same for this, we only do this in auto, so no algae in claw
  CORAL_INTAKE_ASSIST_FLOOR_CLAW_EMPTY,

  ALGAE_INTAKE_FLOOR_DEPLOY_EMPTY,
  ALGAE_INTAKE_L2_LEFT_DEPLOY_EMPTY,
  ALGAE_INTAKE_L3_LEFT_DEPLOY_EMPTY,
  ALGAE_INTAKE_L2_RIGHT_DEPLOY_EMPTY,
  ALGAE_INTAKE_L3_RIGHT_DEPLOY_EMPTY,

  ALGAE_INTAKE_FLOOR_DEPLOY_CORAL,
  ALGAE_INTAKE_L2_LEFT_DEPLOY_CORAL,
  ALGAE_INTAKE_L3_LEFT_DEPLOY_CORAL,
  ALGAE_INTAKE_L2_RIGHT_DEPLOY_CORAL,
  ALGAE_INTAKE_L3_RIGHT_DEPLOY_CORAL,

  // L1 scoring using the ground intake
  CORAL_L1_DEPLOY_PREPARE_CLAW_ALGAE,
  CORAL_L1_DEPLOY_PREPARE_CLAW_EMPTY,
  CORAL_L1_DEPLOY_SCORE_CLAW_ALGAE,
  CORAL_L1_DEPLOY_SCORE_CLAW_EMPTY,

  // L1 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L1_PREPARE_HANDOFF,
  CORAL_L1_RELEASE_HANDOFF,
  /** Coral is in the claw, let's get ready to score L1. */
  CORAL_L1_APPROACH,
  CORAL_L1_LEFT_LINEUP,
  CORAL_L1_RIGHT_LINEUP,
  CORAL_L1_LEFT_RELEASE,
  CORAL_L1_RIGHT_RELEASE,

  // L2 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L2_PREPARE_HANDOFF,
  CORAL_L2_RELEASE_HANDOFF,

  /** Coral is in the claw, let's get ready to score L2. */
  CORAL_L2_APPROACH,
  CORAL_L2_LEFT_LINEUP,
  CORAL_L2_RIGHT_LINEUP,
  CORAL_L2_LEFT_RELEASE,
  CORAL_L2_RIGHT_RELEASE,

  // L3 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L3_PREPARE_HANDOFF,
  CORAL_L3_RELEASE_HANDOFF,
  /** Coral is in the claw, let's get ready to score L3. */
  CORAL_L3_APPROACH,
  CORAL_L3_LEFT_LINEUP,
  CORAL_L3_RIGHT_LINEUP,
  CORAL_L3_LEFT_RELEASE,
  CORAL_L3_RIGHT_RELEASE,

  // L4 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L4_PREPARE_HANDOFF,
  CORAL_L4_RELEASE_HANDOFF,
  /** Coral is in the claw, let's get ready to score L4. */
  CORAL_L4_APPROACH,
  CORAL_L4_LEFT_LINEUP,
  CORAL_L4_RIGHT_LINEUP,
  CORAL_L4_LEFT_RELEASE,
  CORAL_L4_RIGHT_RELEASE,

  // Algae scoring states
  ALGAE_NET_LEFT_WAITING_DEPLOY_EMPTY,
  ALGAE_NET_LEFT_RELEASE_DEPLOY_EMPTY,
  ALGAE_NET_LEFT_WAITING_DEPLOY_CORAL,
  ALGAE_NET_LEFT_RELEASE_DEPLOY_CORAL,

  ALGAE_NET_RIGHT_WAITING_DEPLOY_EMPTY,
  ALGAE_NET_RIGHT_RELEASE_DEPLOY_EMPTY,
  ALGAE_NET_RIGHT_WAITING_DEPLOY_CORAL,
  ALGAE_NET_RIGHT_RELEASE_DEPLOY_CORAL,

  ALGAE_PROCESSOR_WAITING_DEPLOY_EMPTY,
  ALGAE_PROCESSOR_RELEASE_DEPLOY_EMPTY,
  ALGAE_PROCESSOR_WAITING_DEPLOY_CORAL,
  ALGAE_PROCESSOR_RELEASE_DEPLOY_CORAL,

  // Climbing states
  CLIMBING_1_LINEUP,
  CLIMBING_2_HANGING,

  // Misc states
  UNJAM,
  REHOME_DEPLOY,
  REHOME_ELEVATOR;
}
