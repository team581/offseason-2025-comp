package frc.robot.robot_manager;

import com.google.common.collect.ImmutableMap;

public enum RobotState {
  STARTING_POSITION(false, ClawGamePiece.NONE),
  STARTING_POSITION_CORAL(false, ClawGamePiece.CORAL),
  // Stowed states
  CLAW_EMPTY(false, ClawGamePiece.NONE),
  CLAW_CORAL(false, ClawGamePiece.CORAL),
  CLAW_ALGAE(false, ClawGamePiece.ALGAE),

  // Coral intake
  CORAL_INTAKE_GROUND(false, ClawGamePiece.NONE),

  CORAL_OUTTAKE(false, ClawGamePiece.CORAL),

  // Coral score L1
  CORAL_L1_APPROACH(false, ClawGamePiece.CORAL),
  CORAL_L1_LINEUP(false, ClawGamePiece.CORAL),
  CORAL_L1_RELEASE(false, ClawGamePiece.CORAL),

  // Algae intake states
  ALGAE_INTAKE_FLOOR(false, ClawGamePiece.NONE),

  ALGAE_INTAKE_L2(false, ClawGamePiece.NONE),
  ALGAE_INTAKE_L3(false, ClawGamePiece.NONE),

  ALGAE_INTAKE_L2_APPROACH(false, ClawGamePiece.NONE),
  ALGAE_INTAKE_L3_APPROACH(false, ClawGamePiece.NONE),

  ALGAE_INTAKE_L2_HOLDING(false, ClawGamePiece.ALGAE),
  ALGAE_INTAKE_L3_HOLDING(false, ClawGamePiece.ALGAE),

  ALGAE_OUTTAKE(false, ClawGamePiece.ALGAE),

  // Algae scoring states
  ALGAE_NET_WAITING(false, ClawGamePiece.ALGAE),
  ALGAE_NET_RELEASE(false, ClawGamePiece.ALGAE),

  ALGAE_PROCESSOR_WAITING(false, ClawGamePiece.ALGAE),
  ALGAE_PROCESSOR_RELEASE(false, ClawGamePiece.ALGAE),

  // Climbing states
  CLIMBING_1_LINEUP(true, ClawGamePiece.NONE),
  CLIMBING_2_HANGING(true, ClawGamePiece.NONE),
  CLIMBER_STOP(true, ClawGamePiece.NONE),

  // Misc states
  REHOME_ELEVATOR(true, ClawGamePiece.NONE),
  REHOME_WRIST(true, ClawGamePiece.NONE),
  UNJAM(false, ClawGamePiece.NONE);

  public final boolean climbingOrRehoming;
  public final ClawGamePiece heldGamePiece;

  private RobotState(boolean climbingOrRehoming, ClawGamePiece heldGamePiece) {
    this.climbingOrRehoming = climbingOrRehoming;
    this.heldGamePiece = heldGamePiece;
  }

  private static final ImmutableMap<RobotState, RobotState> ALGAE_INTAKE_SEQUENCE =
      ImmutableMap.of(
          ALGAE_INTAKE_L2_APPROACH,
          ALGAE_INTAKE_L2,
          ALGAE_INTAKE_L3_APPROACH,
          ALGAE_INTAKE_L3,
          ALGAE_INTAKE_L2,
          ALGAE_INTAKE_L2_HOLDING,
          ALGAE_INTAKE_L3,
          ALGAE_INTAKE_L3_HOLDING);

  public RobotState getNextAlgaeIntakeState() {
    return ALGAE_INTAKE_SEQUENCE.getOrDefault(this, this);
  }
}
