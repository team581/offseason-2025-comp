package frc.robot.robot_manager;

import frc.robot.auto_align.RobotScoringSide;
import java.util.Map;

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

  // Intake states
  CORAL_INTAKE_FLOOR_CLAW_EMPTY,
  CORAL_INTAKE_FLOOR_CLAW_ALGAE,
  // In theory we could have intake upright while holding algae but nobody is going to use that
  CORAL_INTAKE_LOLLIPOP_CLAW_EMPTY,
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

  private static final Map<RobotState, RobotState> lineupToRelease =
      Map.ofEntries(
          Map.entry(CORAL_L1_LEFT_LINEUP, CORAL_L1_LEFT_RELEASE),
          Map.entry(CORAL_L2_LEFT_LINEUP, CORAL_L2_LEFT_RELEASE),
          Map.entry(CORAL_L3_LEFT_LINEUP, CORAL_L3_LEFT_RELEASE),
          Map.entry(CORAL_L4_LEFT_LINEUP, CORAL_L4_LEFT_RELEASE),
          Map.entry(CORAL_L1_RIGHT_LINEUP, CORAL_L1_RIGHT_RELEASE),
          Map.entry(CORAL_L2_RIGHT_LINEUP, CORAL_L2_RIGHT_RELEASE),
          Map.entry(CORAL_L3_RIGHT_LINEUP, CORAL_L3_RIGHT_RELEASE),
          Map.entry(CORAL_L4_RIGHT_LINEUP, CORAL_L4_RIGHT_RELEASE));

  private static final Map<RobotState, RobotState> algaeAfterScore =
      Map.ofEntries(
          Map.entry(ALGAE_PROCESSOR_RELEASE_DEPLOY_CORAL, CLAW_EMPTY_DEPLOY_CORAL),
          Map.entry(ALGAE_NET_LEFT_RELEASE_DEPLOY_CORAL, CLAW_EMPTY_DEPLOY_CORAL),
          Map.entry(ALGAE_NET_RIGHT_RELEASE_DEPLOY_CORAL, CLAW_EMPTY_DEPLOY_CORAL),
          Map.entry(ALGAE_PROCESSOR_RELEASE_DEPLOY_EMPTY, CLAW_EMPTY_DEPLOY_EMPTY),
          Map.entry(ALGAE_NET_LEFT_RELEASE_DEPLOY_EMPTY, CLAW_EMPTY_DEPLOY_EMPTY),
          Map.entry(ALGAE_NET_RIGHT_RELEASE_DEPLOY_EMPTY, CLAW_EMPTY_DEPLOY_EMPTY));
  private static final Map<RobotState, RobotState> algaeAfterIntake =
      Map.ofEntries(
          Map.entry(ALGAE_INTAKE_L2_LEFT_DEPLOY_EMPTY, CLAW_ALGAE_DEPLOY_EMPTY),
          Map.entry(ALGAE_INTAKE_L3_LEFT_DEPLOY_EMPTY, CLAW_ALGAE_DEPLOY_EMPTY),
          Map.entry(ALGAE_INTAKE_L2_RIGHT_DEPLOY_EMPTY, CLAW_ALGAE_DEPLOY_EMPTY),
          Map.entry(ALGAE_INTAKE_L3_RIGHT_DEPLOY_EMPTY, CLAW_ALGAE_DEPLOY_EMPTY),
          Map.entry(ALGAE_INTAKE_L2_LEFT_DEPLOY_CORAL, CLAW_ALGAE_DEPLOY_CORAL),
          Map.entry(ALGAE_INTAKE_L3_LEFT_DEPLOY_CORAL, CLAW_ALGAE_DEPLOY_CORAL),
          Map.entry(ALGAE_INTAKE_L2_RIGHT_DEPLOY_CORAL, CLAW_ALGAE_DEPLOY_CORAL),
          Map.entry(ALGAE_INTAKE_L3_RIGHT_DEPLOY_CORAL, CLAW_ALGAE_DEPLOY_CORAL));
  private static final Map<RobotState, RobotState> algaeAfterFloorIntake =
      Map.ofEntries(
          Map.entry(ALGAE_INTAKE_FLOOR_DEPLOY_EMPTY, CLAW_ALGAE_DEPLOY_EMPTY),
          Map.entry(ALGAE_INTAKE_FLOOR_DEPLOY_CORAL, CLAW_ALGAE_DEPLOY_CORAL));
  private static final Map<RobotState, RobotState> coralAfterIntake =
      Map.ofEntries(
          Map.entry(CORAL_INTAKE_FLOOR_CLAW_EMPTY, CLAW_EMPTY_DEPLOY_CORAL),
          Map.entry(CORAL_INTAKE_LOLLIPOP_CLAW_EMPTY, CLAW_CORAL_DEPLOY_EMPTY),
          Map.entry(CORAL_INTAKE_ASSIST_FLOOR_CLAW_EMPTY, CLAW_EMPTY_DEPLOY_CORAL),
          Map.entry(CORAL_INTAKE_FLOOR_CLAW_ALGAE, CLAW_ALGAE_DEPLOY_CORAL));
  private static final Map<RobotState, RobotState> handoffPrepareToRelease =
      Map.ofEntries(
          Map.entry(CORAL_L1_PREPARE_HANDOFF, CORAL_L1_RELEASE_HANDOFF),
          Map.entry(CORAL_L2_PREPARE_HANDOFF, CORAL_L2_RELEASE_HANDOFF),
          Map.entry(CORAL_L3_PREPARE_HANDOFF, CORAL_L3_RELEASE_HANDOFF),
          Map.entry(CORAL_L4_PREPARE_HANDOFF, CORAL_L4_RELEASE_HANDOFF));
  private static final Map<RobotState, RobotState> handoffReleaseToApproach =
      Map.ofEntries(
          Map.entry(CORAL_L1_RELEASE_HANDOFF, CORAL_L1_APPROACH),
          Map.entry(CORAL_L2_RELEASE_HANDOFF, CORAL_L2_APPROACH),
          Map.entry(CORAL_L3_RELEASE_HANDOFF, CORAL_L3_APPROACH),
          Map.entry(CORAL_L4_RELEASE_HANDOFF, CORAL_L4_APPROACH));
  private static final Map<RobotState, RobotState> approachToLineupLeftStates =
      Map.ofEntries(
          Map.entry(CORAL_L1_APPROACH, CORAL_L1_LEFT_LINEUP),
          Map.entry(CORAL_L2_APPROACH, CORAL_L2_LEFT_LINEUP),
          Map.entry(CORAL_L3_APPROACH, CORAL_L3_LEFT_LINEUP),
          Map.entry(CORAL_L4_APPROACH, CORAL_L4_LEFT_LINEUP));
  private static final Map<RobotState, RobotState> approachToLineupRightStates =
      Map.ofEntries(
          Map.entry(CORAL_L1_APPROACH, CORAL_L1_RIGHT_LINEUP),
          Map.entry(CORAL_L2_APPROACH, CORAL_L2_RIGHT_LINEUP),
          Map.entry(CORAL_L3_APPROACH, CORAL_L3_RIGHT_LINEUP),
          Map.entry(CORAL_L4_APPROACH, CORAL_L4_RIGHT_LINEUP));
  private static final Map<RobotState, RobotState> deployScorePrepareToReleaseStates =
      Map.ofEntries(
          Map.entry(CORAL_L1_DEPLOY_PREPARE_CLAW_EMPTY, CORAL_L1_DEPLOY_SCORE_CLAW_EMPTY),
          Map.entry(CORAL_L1_DEPLOY_PREPARE_CLAW_ALGAE, CORAL_L1_DEPLOY_SCORE_CLAW_ALGAE));
  private static final Map<RobotState, RobotState> afterDeployScore =
      Map.ofEntries(
          Map.entry(CORAL_L1_DEPLOY_SCORE_CLAW_EMPTY, CLAW_EMPTY_DEPLOY_EMPTY),
          Map.entry(CORAL_L1_DEPLOY_SCORE_CLAW_ALGAE, CLAW_ALGAE_DEPLOY_EMPTY));

  public RobotState getLineupToReleaseState() {
    return lineupToRelease.getOrDefault(this, this);
  }

  public RobotState getAlgaeAfterScore() {
    return algaeAfterScore.getOrDefault(this, this);
  }

  public RobotState getAlgaeAfterReefIntake() {
    return algaeAfterIntake.getOrDefault(this, this);
  }

  public RobotState getAlgaeAfterFloorIntake() {
    return algaeAfterFloorIntake.getOrDefault(this, this);
  }

  public RobotState getCoralAfterIntake() {
    return coralAfterIntake.getOrDefault(this, this);
  }

  public RobotState getHandoffPrepareToReleaseState() {
    return handoffPrepareToRelease.getOrDefault(this, this);
  }

  public RobotState getHandoffReleaseToApproachState() {
    return handoffReleaseToApproach.getOrDefault(this, this);
  }

  public RobotState getApproachToLineupState(RobotScoringSide scoringSide) {
    var map =
        scoringSide == RobotScoringSide.LEFT
            ? approachToLineupLeftStates
            : approachToLineupRightStates;
    return map.getOrDefault(this, this);
  }

  public RobotState getDeployScorePrepareToRelease() {
    return deployScorePrepareToReleaseStates.getOrDefault(this, this);
  }

  public RobotState getAfterDeployScore() {
    return afterDeployScore.getOrDefault(this, this);
  }
}
