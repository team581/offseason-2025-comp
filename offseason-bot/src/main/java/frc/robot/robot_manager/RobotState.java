package frc.robot.robot_manager;

import com.google.common.collect.ImmutableMap;
import java.util.Map;

public enum RobotState {
  // Idle states
  /** Idle without any game piece. */
  CLAW_EMPTY(ClawGamePiece.EMPTY, false),
  /** Claw holding algae */
  CLAW_ALGAE(ClawGamePiece.ALGAE, false),
  /** Claw holding coral */
  CLAW_CORAL(ClawGamePiece.CORAL, false),
  /** Claw empty, arm upright */
  STARTING_POSITION(ClawGamePiece.EMPTY, false),
  /** Claw holding coral, arm upright */
  STARTING_POSITION_CORAL(ClawGamePiece.CORAL, false),
  /** Claw holding algae, stowed inward */
  CLAW_ALGAE_STOW_INWARD(ClawGamePiece.ALGAE, false),

  // Intake states
  // In theory we could have intake upright while holding algae but nobody is going to use that
  CORAL_INTAKE_LOLLIPOP_APPROACH(ClawGamePiece.EMPTY, false),
  CORAL_INTAKE_LOLLIPOP_PUSH(ClawGamePiece.CORAL, false),
  CORAL_INTAKE_LOLLIPOP_GRAB(ClawGamePiece.EMPTY, false),

  ALGAE_INTAKE_FLOOR(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L2_APPROACH(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L3_APPROACH(ClawGamePiece.ALGAE, false),

  ALGAE_INTAKE_L2_HOLDING(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L3_HOLDING(ClawGamePiece.ALGAE, false),

  ALGAE_INTAKE_L2(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L3(ClawGamePiece.ALGAE, false),

  ALGAE_FLING_WAIT(ClawGamePiece.ALGAE, false),
  ALGAE_FLING_PREPARE(ClawGamePiece.ALGAE, false),
  ALGAE_FLING_RELEASE(ClawGamePiece.ALGAE, false),

  // L1 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L1_PREPARE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L1_RELEASE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L1_AFTER_RELEASE_HANDOFF(ClawGamePiece.CORAL, false),

  /** Coral is in the claw, let's get ready to score L1. */
  CORAL_L1_APPROACH(ClawGamePiece.CORAL, false),
  CORAL_L1_LINEUP(ClawGamePiece.CORAL, false),
  CORAL_L1_RELEASE(ClawGamePiece.CORAL, false),

  // L2 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L2_PREPARE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L2_RELEASE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L2_AFTER_RELEASE_HANDOFF(ClawGamePiece.CORAL, false),

  /** Coral is in the claw, let's get ready to score L2. */
  CORAL_L2_APPROACH(ClawGamePiece.CORAL, false),

  CORAL_L2_LINEUP(ClawGamePiece.CORAL, false),
  CORAL_L2_PLACE(ClawGamePiece.CORAL, false),
  CORAL_L2_RELEASE(ClawGamePiece.CORAL, false),

  // L3 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L3_PREPARE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L3_RELEASE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L3_AFTER_RELEASE_HANDOFF(ClawGamePiece.CORAL, false),

  /** Coral is in the claw, let's get ready to score L3. */
  CORAL_L3_APPROACH(ClawGamePiece.CORAL, false),
  CORAL_L3_LINEUP(ClawGamePiece.CORAL, false),
  CORAL_L3_PLACE(ClawGamePiece.CORAL, false),
  CORAL_L3_RELEASE(ClawGamePiece.CORAL, false),

  // L4 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L4_PREPARE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L4_RELEASE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L4_AFTER_RELEASE_HANDOFF(ClawGamePiece.CORAL, false),

  /** Coral is in the claw, let's get ready to score L4. */
  CORAL_L4_APPROACH(ClawGamePiece.CORAL, false),
  CORAL_L4_LINEUP(ClawGamePiece.CORAL, false),
  CORAL_L4_PLACE(ClawGamePiece.CORAL, false),
  CORAL_L4_RELEASE(ClawGamePiece.CORAL, false),

  // Algae scoring states
  ALGAE_NET_WAITING(ClawGamePiece.ALGAE, false),
  ALGAE_NET_RELEASE(ClawGamePiece.ALGAE, false),

  ALGAE_PROCESSOR_WAITING(ClawGamePiece.ALGAE, false),
  ALGAE_PROCESSOR_RELEASE(ClawGamePiece.ALGAE, false),

  // Climbing states
  CLIMBING_1_LINEUP(ClawGamePiece.EMPTY, true),
  CLIMBING_2_HANGING(ClawGamePiece.EMPTY, true),
  CLIMBER_STOP(ClawGamePiece.EMPTY, true),

  // Misc states
  LOW_STOW(ClawGamePiece.EMPTY, false),
  PREPARE_SPIN_TO_WIN(ClawGamePiece.EMPTY, false),
  SPIN_TO_WIN(ClawGamePiece.EMPTY, false),
  ALGAE_OUTTAKE(ClawGamePiece.ALGAE, false),
  UNJAM(ClawGamePiece.EMPTY, false),
  REHOME_ELEVATOR(ClawGamePiece.EMPTY, true),

  FORCED_HANDOFF(ClawGamePiece.EMPTY, false),
  FORCED_LOWSTOW(ClawGamePiece.EMPTY, false);

  public final ClawGamePiece clawGp;
  public final boolean climbingOrRehoming;

  private RobotState(ClawGamePiece clawGp, boolean climbingOrRehoming) {
    this.clawGp = clawGp;
    this.climbingOrRehoming = climbingOrRehoming;
  }

  private static final ImmutableMap<RobotState, RobotState> scoreSequence =
      ImmutableMap.ofEntries(
          Map.entry(CORAL_L1_LINEUP, CORAL_L1_RELEASE),
          Map.entry(CORAL_L2_LINEUP, CORAL_L2_PLACE),
          Map.entry(CORAL_L3_LINEUP, CORAL_L3_PLACE),
          Map.entry(CORAL_L4_LINEUP, CORAL_L4_PLACE),
          Map.entry(CORAL_L1_RELEASE, CORAL_L1_RELEASE),
          Map.entry(CORAL_L2_PLACE, CORAL_L2_RELEASE),
          Map.entry(CORAL_L3_PLACE, CORAL_L3_RELEASE),
          Map.entry(CORAL_L4_PLACE, CORAL_L4_RELEASE));

  private static final ImmutableMap<RobotState, RobotState> handoffSequence =
      ImmutableMap.ofEntries(
          Map.entry(CORAL_L1_PREPARE_HANDOFF, CORAL_L1_RELEASE_HANDOFF),
          Map.entry(CORAL_L2_PREPARE_HANDOFF, CORAL_L2_RELEASE_HANDOFF),
          Map.entry(CORAL_L3_PREPARE_HANDOFF, CORAL_L3_RELEASE_HANDOFF),
          Map.entry(CORAL_L4_PREPARE_HANDOFF, CORAL_L4_RELEASE_HANDOFF),
          Map.entry(CORAL_L1_RELEASE_HANDOFF, CORAL_L1_AFTER_RELEASE_HANDOFF),
          Map.entry(CORAL_L2_RELEASE_HANDOFF, CORAL_L2_AFTER_RELEASE_HANDOFF),
          Map.entry(CORAL_L3_RELEASE_HANDOFF, CORAL_L3_AFTER_RELEASE_HANDOFF),
          Map.entry(CORAL_L4_RELEASE_HANDOFF, CORAL_L4_AFTER_RELEASE_HANDOFF));
  private static final ImmutableMap<RobotState, RobotState> handoffAfterReleaseToApproach =
      ImmutableMap.ofEntries(
          Map.entry(CORAL_L1_AFTER_RELEASE_HANDOFF, CORAL_L1_APPROACH),
          Map.entry(CORAL_L2_AFTER_RELEASE_HANDOFF, CORAL_L2_APPROACH),
          Map.entry(CORAL_L3_AFTER_RELEASE_HANDOFF, CORAL_L3_APPROACH),
          Map.entry(CORAL_L4_AFTER_RELEASE_HANDOFF, CORAL_L4_APPROACH));

  private static final ImmutableMap<RobotState, RobotState> algaeIntakeSequence =
      ImmutableMap.ofEntries(
          Map.entry(ALGAE_INTAKE_L2_APPROACH, ALGAE_INTAKE_L2),
          Map.entry(ALGAE_INTAKE_L3_APPROACH, ALGAE_INTAKE_L3),
          Map.entry(ALGAE_INTAKE_L2, ALGAE_INTAKE_L2_HOLDING),
          Map.entry(ALGAE_INTAKE_L3, ALGAE_INTAKE_L3_HOLDING));

  public static boolean isLineupOrApproachState(RobotState state) {
    return switch (state) {
      case CORAL_L4_APPROACH,
          CORAL_L4_LINEUP,
          CORAL_L3_APPROACH,
          CORAL_L3_LINEUP,
          CORAL_L2_APPROACH,
          CORAL_L2_LINEUP,
          CORAL_L1_LINEUP,
          CORAL_L1_APPROACH ->
          true;
      default -> false;
    };
  }

  public static boolean isReleaseState(RobotState state) {
    return switch (state) {
      case CORAL_L1_RELEASE, CORAL_L2_RELEASE, CORAL_L3_RELEASE, CORAL_L4_RELEASE -> true;
      default -> false;
    };
  }

  public RobotState getNextScoreState() {
    return scoreSequence.getOrDefault(this, this);
  }

  public RobotState getNextAlgaeIntakeState() {
    return algaeIntakeSequence.getOrDefault(this, this);
  }

  public RobotState getNextHandoffState() {
    return handoffSequence.getOrDefault(this, this);
  }

  public RobotState getHandoffAfterReleaseToApproachState() {
    return handoffAfterReleaseToApproach.getOrDefault(this, this);
  }
}
