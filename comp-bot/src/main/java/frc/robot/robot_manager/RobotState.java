package frc.robot.robot_manager;

import com.google.common.collect.ImmutableMap;
import frc.robot.auto_align.RobotScoringSide;
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
  ALGAE_INTAKE_L2_LEFT_APPROACH(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L3_LEFT_APPROACH(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L2_RIGHT_APPROACH(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L3_RIGHT_APPROACH(ClawGamePiece.ALGAE, false),

  ALGAE_INTAKE_L2_LEFT_HOLDING(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L3_LEFT_HOLDING(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L2_RIGHT_HOLDING(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L3_RIGHT_HOLDING(ClawGamePiece.ALGAE, false),

  ALGAE_INTAKE_L2_LEFT(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L3_LEFT(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L2_RIGHT(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L3_RIGHT(ClawGamePiece.ALGAE, false),

  ALGAE_FLING_WAIT(ClawGamePiece.ALGAE, false),
  ALGAE_FLING_PREPARE(ClawGamePiece.ALGAE, false),
  ALGAE_FLING_RELEASE(ClawGamePiece.ALGAE, false),

  // L1 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L1_PREPARE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L1_RELEASE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L1_AFTER_HANDOFF(ClawGamePiece.CORAL, false),

  /** Coral is in the claw, let's get ready to score L1. */
  CORAL_L1_RIGHT_APPROACH(ClawGamePiece.CORAL, false),
  CORAL_L1_RIGHT_LINEUP(ClawGamePiece.CORAL, false),
  CORAL_L1_RIGHT_RELEASE(ClawGamePiece.CORAL, false),

  // L2 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L2_PREPARE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L2_RELEASE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L2_AFTER_HANDOFF(ClawGamePiece.CORAL, false),

  /** Coral is in the claw, let's get ready to score L2. */
  CORAL_L2_LEFT_APPROACH(ClawGamePiece.CORAL, false),
  CORAL_L2_RIGHT_APPROACH(ClawGamePiece.CORAL, false),

  CORAL_L2_LEFT_LINEUP(ClawGamePiece.CORAL, false),
  CORAL_L2_RIGHT_LINEUP(ClawGamePiece.CORAL, false),
  CORAL_L2_LEFT_PLACE(ClawGamePiece.CORAL, false),
  CORAL_L2_RIGHT_PLACE(ClawGamePiece.CORAL, false),
  CORAL_L2_LEFT_RELEASE(ClawGamePiece.CORAL, false),
  CORAL_L2_RIGHT_RELEASE(ClawGamePiece.CORAL, false),

  // L3 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L3_PREPARE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L3_RELEASE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L3_AFTER_HANDOFF(ClawGamePiece.CORAL, false),

  /** Coral is in the claw, let's get ready to score L3. */
  CORAL_L3_LEFT_APPROACH(ClawGamePiece.CORAL, false),
  CORAL_L3_RIGHT_APPROACH(ClawGamePiece.CORAL, false),
  CORAL_L3_LEFT_LINEUP(ClawGamePiece.CORAL, false),
  CORAL_L3_RIGHT_LINEUP(ClawGamePiece.CORAL, false),
  CORAL_L3_LEFT_PLACE(ClawGamePiece.CORAL, false),
  CORAL_L3_RIGHT_PLACE(ClawGamePiece.CORAL, false),
  CORAL_L3_LEFT_RELEASE(ClawGamePiece.CORAL, false),
  CORAL_L3_RIGHT_RELEASE(ClawGamePiece.CORAL, false),

  // L4 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L4_PREPARE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L4_RELEASE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L4_AFTER_HANDOFF(ClawGamePiece.CORAL, false),

  /** Coral is in the claw, let's get ready to score L4. */
  CORAL_L4_LEFT_APPROACH(ClawGamePiece.CORAL, false),
  CORAL_L4_RIGHT_APPROACH(ClawGamePiece.CORAL, false),
  CORAL_L4_LEFT_LINEUP(ClawGamePiece.CORAL, false),
  CORAL_L4_RIGHT_LINEUP(ClawGamePiece.CORAL, false),
  CORAL_L4_LEFT_PLACE(ClawGamePiece.CORAL, false),
  CORAL_L4_RIGHT_PLACE(ClawGamePiece.CORAL, false),
  CORAL_L4_LEFT_RELEASE(ClawGamePiece.CORAL, false),
  CORAL_L4_RIGHT_RELEASE(ClawGamePiece.CORAL, false),

  // Algae scoring states
  ALGAE_NET_LEFT_WAITING(ClawGamePiece.ALGAE, false),
  ALGAE_NET_LEFT_RELEASE(ClawGamePiece.ALGAE, false),

  ALGAE_NET_RIGHT_WAITING(ClawGamePiece.ALGAE, false),
  ALGAE_NET_RIGHT_RELEASE(ClawGamePiece.ALGAE, false),

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

  FORCED_HANDOFF(ClawGamePiece.EMPTY, false),
  FORCED_LOWSTOW(ClawGamePiece.EMPTY, false);

  public final ClawGamePiece clawGp;
  public final boolean climbingOrRehoming;

  private RobotState(ClawGamePiece clawGp, boolean climbingOrRehoming) {
    this.clawGp = clawGp;
    this.climbingOrRehoming = climbingOrRehoming;
  }

  private static final ImmutableMap<RobotState, RobotState> LINEUP_TO_PLACE =
      ImmutableMap.ofEntries(
          Map.entry(CORAL_L2_LEFT_LINEUP, CORAL_L2_LEFT_PLACE),
          Map.entry(CORAL_L3_LEFT_LINEUP, CORAL_L3_LEFT_PLACE),
          Map.entry(CORAL_L4_LEFT_LINEUP, CORAL_L4_LEFT_PLACE),
          Map.entry(CORAL_L1_RIGHT_LINEUP, CORAL_L1_RIGHT_RELEASE),
          Map.entry(CORAL_L2_RIGHT_LINEUP, CORAL_L2_RIGHT_PLACE),
          Map.entry(CORAL_L3_RIGHT_LINEUP, CORAL_L3_RIGHT_PLACE),
          Map.entry(CORAL_L4_RIGHT_LINEUP, CORAL_L4_RIGHT_PLACE));
  private static final ImmutableMap<RobotState, RobotState> PLACE_TO_RELEASE =
      ImmutableMap.ofEntries(
          Map.entry(CORAL_L2_LEFT_PLACE, CORAL_L2_LEFT_RELEASE),
          Map.entry(CORAL_L3_LEFT_PLACE, CORAL_L3_LEFT_RELEASE),
          Map.entry(CORAL_L4_LEFT_PLACE, CORAL_L4_LEFT_RELEASE),
          Map.entry(CORAL_L1_RIGHT_RELEASE, CORAL_L1_RIGHT_RELEASE),
          Map.entry(CORAL_L2_RIGHT_PLACE, CORAL_L2_RIGHT_RELEASE),
          Map.entry(CORAL_L3_RIGHT_PLACE, CORAL_L3_RIGHT_RELEASE),
          Map.entry(CORAL_L4_RIGHT_PLACE, CORAL_L4_RIGHT_RELEASE));

  private static final ImmutableMap<RobotState, RobotState> HANDOFF_PREPARE_TO_RELEASE =
      ImmutableMap.of(
          CORAL_L1_PREPARE_HANDOFF,
          CORAL_L1_RELEASE_HANDOFF,
          CORAL_L2_PREPARE_HANDOFF,
          CORAL_L2_RELEASE_HANDOFF,
          CORAL_L3_PREPARE_HANDOFF,
          CORAL_L3_RELEASE_HANDOFF,
          CORAL_L4_PREPARE_HANDOFF,
          CORAL_L4_RELEASE_HANDOFF);

  private static final ImmutableMap<RobotState, RobotState> HANDOFF_RELEASE_TO_AFTER_HANDOFF =
      ImmutableMap.of(
          CORAL_L1_RELEASE_HANDOFF,
          CORAL_L1_AFTER_HANDOFF,
          CORAL_L2_RELEASE_HANDOFF,
          CORAL_L2_AFTER_HANDOFF,
          CORAL_L3_RELEASE_HANDOFF,
          CORAL_L3_AFTER_HANDOFF,
          CORAL_L4_RELEASE_HANDOFF,
          CORAL_L4_AFTER_HANDOFF);
  private static final ImmutableMap<RobotState, RobotState> AFTER_HANDOFF_TO_LEFT_APPROACH =
      ImmutableMap.of(
          CORAL_L1_AFTER_HANDOFF,
          CORAL_L1_RIGHT_APPROACH,
          CORAL_L2_AFTER_HANDOFF,
          CORAL_L2_LEFT_APPROACH,
          CORAL_L3_AFTER_HANDOFF,
          CORAL_L3_LEFT_APPROACH,
          CORAL_L4_AFTER_HANDOFF,
          CORAL_L4_LEFT_APPROACH);
  private static final ImmutableMap<RobotState, RobotState> AFTER_HANDOFF_TO_RIGHT_APPROACH =
      ImmutableMap.of(
          CORAL_L1_AFTER_HANDOFF,
          CORAL_L1_RIGHT_APPROACH,
          CORAL_L2_AFTER_HANDOFF,
          CORAL_L2_RIGHT_APPROACH,
          CORAL_L3_AFTER_HANDOFF,
          CORAL_L3_RIGHT_APPROACH,
          CORAL_L4_AFTER_HANDOFF,
          CORAL_L4_RIGHT_APPROACH);
  private static final ImmutableMap<RobotState, RobotState> APPROACH_TO_LINEUP_LEFT_STATES =
      ImmutableMap.ofEntries(
          // Go to right l1 since robot can't do left l1
          Map.entry(CORAL_L1_RIGHT_APPROACH, CORAL_L1_RIGHT_LINEUP),
          Map.entry(CORAL_L2_LEFT_APPROACH, CORAL_L2_LEFT_LINEUP),
          Map.entry(CORAL_L3_LEFT_APPROACH, CORAL_L3_LEFT_LINEUP),
          Map.entry(CORAL_L4_LEFT_APPROACH, CORAL_L4_LEFT_LINEUP));

  private static final ImmutableMap<RobotState, RobotState> ALGAE_APPROACH_TO_INTAKE_STATES =
      ImmutableMap.ofEntries(
          // Go to right l1 since robot can't do left l1
          Map.entry(ALGAE_INTAKE_L2_LEFT_APPROACH, ALGAE_INTAKE_L2_LEFT),
          Map.entry(ALGAE_INTAKE_L3_LEFT_APPROACH, ALGAE_INTAKE_L3_LEFT),
          Map.entry(ALGAE_INTAKE_L2_RIGHT_APPROACH, ALGAE_INTAKE_L2_RIGHT),
          Map.entry(ALGAE_INTAKE_L3_RIGHT_APPROACH, ALGAE_INTAKE_L3_RIGHT));

  private static final ImmutableMap<RobotState, RobotState> ALGAE_INTAKE_TO_HOLDING_STATES =
      ImmutableMap.ofEntries(
          // Go to right l1 since robot can't do left l1
          Map.entry(ALGAE_INTAKE_L2_LEFT, ALGAE_INTAKE_L2_LEFT_HOLDING),
          Map.entry(ALGAE_INTAKE_L3_LEFT, ALGAE_INTAKE_L3_LEFT_HOLDING),
          Map.entry(ALGAE_INTAKE_L2_RIGHT, ALGAE_INTAKE_L2_RIGHT_HOLDING),
          Map.entry(ALGAE_INTAKE_L3_RIGHT, ALGAE_INTAKE_L3_RIGHT_HOLDING));
  private static final ImmutableMap<RobotState, RobotState> APPROACH_TO_LINEUP_RIGHT_STATES =
      ImmutableMap.of(
          CORAL_L1_RIGHT_APPROACH,
          CORAL_L1_RIGHT_LINEUP,
          CORAL_L2_RIGHT_APPROACH,
          CORAL_L2_RIGHT_LINEUP,
          CORAL_L3_RIGHT_APPROACH,
          CORAL_L3_RIGHT_LINEUP,
          CORAL_L4_RIGHT_APPROACH,
          CORAL_L4_RIGHT_LINEUP);

  private static final ImmutableMap<RobotState, RobotState> RIGHT_TO_LEFT_APPROACH_STATES =
      ImmutableMap.of(
          CORAL_L2_RIGHT_APPROACH,
          CORAL_L2_LEFT_APPROACH,
          CORAL_L3_RIGHT_APPROACH,
          CORAL_L3_LEFT_APPROACH,
          CORAL_L4_RIGHT_APPROACH,
          CORAL_L4_LEFT_APPROACH);

  private static final ImmutableMap<RobotState, RobotState> LEFT_TO_RIGHT_APPROACH_STATES =
      ImmutableMap.of(
          CORAL_L2_LEFT_APPROACH,
          CORAL_L2_RIGHT_APPROACH,
          CORAL_L3_LEFT_APPROACH,
          CORAL_L3_RIGHT_APPROACH,
          CORAL_L4_LEFT_APPROACH,
          CORAL_L4_RIGHT_APPROACH);

  public static boolean isLineupOrApproachState(RobotState state) {
    return switch (state) {
      case CORAL_L4_LEFT_APPROACH,
          CORAL_L4_RIGHT_APPROACH,
          CORAL_L4_LEFT_LINEUP,
          CORAL_L4_RIGHT_LINEUP,
          CORAL_L3_LEFT_APPROACH,
          CORAL_L3_RIGHT_APPROACH,
          CORAL_L3_LEFT_LINEUP,
          CORAL_L3_RIGHT_LINEUP,
          CORAL_L2_LEFT_APPROACH,
          CORAL_L2_RIGHT_APPROACH,
          CORAL_L2_LEFT_LINEUP,
          CORAL_L2_RIGHT_LINEUP,
          CORAL_L1_RIGHT_LINEUP,
          CORAL_L1_RIGHT_APPROACH ->
          true;
      default -> false;
    };
  }

  public static boolean isReleaseState(RobotState state) {
    return switch (state) {
      case CORAL_L1_RIGHT_RELEASE,
          CORAL_L2_LEFT_RELEASE,
          CORAL_L2_RIGHT_RELEASE,
          CORAL_L3_LEFT_RELEASE,
          CORAL_L3_RIGHT_RELEASE,
          CORAL_L4_LEFT_RELEASE,
          CORAL_L4_RIGHT_RELEASE ->
          true;
      default -> false;
    };
  }

  public RobotState getLineupToPlaceState() {
    return LINEUP_TO_PLACE.getOrDefault(this, this);
  }

  public RobotState getPlaceToReleaseState() {
    return PLACE_TO_RELEASE.getOrDefault(this, this);
  }

  public RobotState getAlgaeApproachToIntakeState() {
    return ALGAE_APPROACH_TO_INTAKE_STATES.getOrDefault(this, this);
  }

  public RobotState getAlgaeIntakeToHoldingState() {
    return ALGAE_INTAKE_TO_HOLDING_STATES.getOrDefault(this, this);
  }

  public RobotState getHandoffPrepareToReleaseState() {
    return HANDOFF_PREPARE_TO_RELEASE.getOrDefault(this, this);
  }

  public RobotState getAfterHandoffToApproachState(RobotScoringSide scoringSide) {
    var map =
        scoringSide == RobotScoringSide.LEFT
            ? AFTER_HANDOFF_TO_LEFT_APPROACH
            : AFTER_HANDOFF_TO_RIGHT_APPROACH;
    return map.getOrDefault(this, this);
  }

  public RobotState getHandoffReleaseToAfterHandoffState() {
    return HANDOFF_RELEASE_TO_AFTER_HANDOFF.getOrDefault(this, this);
  }

  public RobotState getLeftApproachToLineupState() {
    return APPROACH_TO_LINEUP_LEFT_STATES.getOrDefault(this, this);
  }

  public RobotState getRightApproachToLineupState() {
    return APPROACH_TO_LINEUP_RIGHT_STATES.getOrDefault(this, this);
  }

  public RobotState getRightToLeftApproachState() {
    return RIGHT_TO_LEFT_APPROACH_STATES.getOrDefault(this, this);
  }

  public RobotState getLeftToRightApproachState() {
    return LEFT_TO_RIGHT_APPROACH_STATES.getOrDefault(this, this);
  }
}
