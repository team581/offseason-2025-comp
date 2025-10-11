package frc.robot.robot_manager;

import com.google.common.collect.ImmutableMap;
import java.util.Map;

public enum RobotState {
  // Stowed states
  CLAW_EMPTY(false),
  CLAW_ALGAE(false),

  // Algae intake states
  ALGAE_INTAKE_FLOOR(false),

  ALGAE_INTAKE_L2(false),
  ALGAE_INTAKE_L3(false),

  ALGAE_INTAKE_L2_APPROACH(false),
  ALGAE_INTAKE_L3_APPROACH(false),

  ALGAE_INTAKE_L2_HOLDING(false),
  ALGAE_INTAKE_L3_HOLDING(false),

  ALGAE_OUTTAKE(false),

  // Algae scoring states
  ALGAE_NET_WAITING(false),
  ALGAE_NET_RELEASE(false),

  ALGAE_PROCESSOR_WAITING(false),
  ALGAE_PROCESSOR_RELEASE(false),

  // Climbing states
  CLIMBING_1_LINEUP(true),
  CLIMBING_2_HANGING(true),
  CLIMBER_STOP(true),

  // Misc states
  REHOME_ELEVATOR(true),
  UNJAM(false);

  public final boolean climbingOrRehoming;

  private RobotState(boolean climbingOrRehoming) {
    this.climbingOrRehoming = climbingOrRehoming;
  }

  private static final ImmutableMap<RobotState, RobotState> algaeIntakeSequence =
      ImmutableMap.ofEntries(
          Map.entry(ALGAE_INTAKE_L2_APPROACH, ALGAE_INTAKE_L2),
          Map.entry(ALGAE_INTAKE_L3_APPROACH, ALGAE_INTAKE_L3),
          Map.entry(ALGAE_INTAKE_L2, ALGAE_INTAKE_L2_HOLDING),
          Map.entry(ALGAE_INTAKE_L3, ALGAE_INTAKE_L3_HOLDING));

  public RobotState getNextAlgaeIntakeState() {
    return algaeIntakeSequence.getOrDefault(this, this);
  }
}
