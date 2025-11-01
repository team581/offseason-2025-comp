package frc.robot.robot_manager.collision_avoidance;

import static java.util.Comparator.comparingDouble;

import com.google.common.collect.ImmutableList;
import com.google.common.graph.MutableValueGraph;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.arm.ArmState;
import frc.robot.config.FeatureFlags;
import frc.robot.elevator.ElevatorState;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * These represent "waypoints" for collision avoidance to route through. These are NOT setpoints
 * that the robot uses, even though they may share a name and/or superstructure position. Collision
 * avoidance uses these as nodes within a graph to route from a current position to a goal position.
 */
public enum Waypoint {
  // STOWED WAYPOINTS
  HANDOFF(new SuperstructurePosition(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF)),
  HANDOFF_CLEARS_CLIMBER(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF.getHeight() + 4.0, ArmState.CORAL_HANDOFF)),
  LOW_STOW(new SuperstructurePosition(ElevatorState.STOWED, ArmState.HOLDING_UPRIGHT)),

  // CLIMB WAYPOINTS
  CLIMB(new SuperstructurePosition(ElevatorState.CLIMBING, ArmState.CLIMBING)),

  // CORAL WAYPOINTS
  CORAL_L2_UPRIGHT(
      new SuperstructurePosition(ElevatorState.CORAL_SCORE_LINEUP_L2, ArmState.HOLDING_UPRIGHT)),
  CORAL_L3_UPRIGHT(
      new SuperstructurePosition(ElevatorState.CORAL_SCORE_LINEUP_L3, ArmState.HOLDING_UPRIGHT)),
  CORAL_L4_UPRIGHT(
      new SuperstructurePosition(ElevatorState.CORAL_SCORE_LINEUP_L4, ArmState.HOLDING_UPRIGHT)),
  CORAL_L2_RIGHT_ARM_ONLY(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_SCORE_RIGHT_LINEUP_L2.getAngle() - 60)),
  CORAL_L2_LEFT_ARM_ONLY(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_SCORE_LEFT_LINEUP_L2.getAngle() + 60)),
  CORAL_L3_RIGHT_ARM_ONLY(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_SCORE_RIGHT_LINEUP_L3.getAngle() - 60)),
  CORAL_L3_LEFT_ARM_ONLY(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_SCORE_LEFT_LINEUP_L3.getAngle() + 60)),

  CORAL_L1_RIGHT_LINEUP(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L1, ArmState.CORAL_SCORE_RIGHT_LINEUP_L1)),
  CORAL_L2_RIGHT_LINEUP(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LINEUP_L2, ArmState.CORAL_SCORE_RIGHT_LINEUP_L2)),
  CORAL_L2_RIGHT_PLACE(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RELEASE_L2, ArmState.CORAL_SCORE_RIGHT_RELEASE_L2)),
  CORAL_L3_RIGHT_LINEUP(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LINEUP_L3, ArmState.CORAL_SCORE_RIGHT_LINEUP_L3)),
  CORAL_L3_RIGHT_PLACE(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RELEASE_L3, ArmState.CORAL_SCORE_RIGHT_RELEASE_L3)),
  CORAL_L4_RIGHT_LINEUP(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LINEUP_L4, ArmState.CORAL_SCORE_RIGHT_LINEUP_L4)),
  CORAL_L4_RIGHT_PLACE(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RELEASE_L4, ArmState.CORAL_SCORE_RIGHT_RELEASE_L4)),
  CORAL_L2_LEFT_LINEUP(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LINEUP_L2, ArmState.CORAL_SCORE_LEFT_LINEUP_L2)),
  CORAL_L2_LEFT_PLACE(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RELEASE_L2, ArmState.CORAL_SCORE_LEFT_RELEASE_L2)),
  CORAL_L3_LEFT_LINEUP(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LINEUP_L3, ArmState.CORAL_SCORE_LEFT_LINEUP_L3)),
  CORAL_L3_LEFT_PLACE(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RELEASE_L3, ArmState.CORAL_SCORE_LEFT_RELEASE_L3)),
  CORAL_L4_LEFT_LINEUP(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LINEUP_L4, ArmState.CORAL_SCORE_LEFT_LINEUP_L4)),
  CORAL_L4_LEFT_PLACE(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RELEASE_L4, ArmState.CORAL_SCORE_LEFT_RELEASE_L4)),

  // ALGAE WAYPOINTS
  ALGAE_GROUND_INTAKE_ARM_ONLY(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF.getHeight(), ArmState.ALGAE_INTAKE_FLOOR)),
  ALGAE_GROUND_INTAKE(
      new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_GROUND, ArmState.ALGAE_INTAKE_FLOOR)),

  ALGAE_INTAKE_L2_RIGHT_ARM_ONLY(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_SCORE_RIGHT_LINEUP_L2.getAngle() - 60)),
  ALGAE_INTAKE_L2_LEFT_ARM_ONLY(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_SCORE_LEFT_LINEUP_L2.getAngle() + 60)),
  ALGAE_INTAKE_L3_RIGHT_ARM_ONLY(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_SCORE_RIGHT_LINEUP_L3.getAngle() - 60)),
  ALGAE_INTAKE_L3_LEFT_ARM_ONLY(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_SCORE_LEFT_LINEUP_L3.getAngle() + 60)),

  ALGAE_INTAKE_L2_RIGHT(
      new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_L2, ArmState.ALGAE_INTAKE_RIGHT_L2)),
  ALGAE_INTAKE_L2_LEFT(
      new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_L2, ArmState.ALGAE_INTAKE_LEFT_L2)),

  ALGAE_INTAKE_L3_RIGHT(
      new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_L3, ArmState.ALGAE_INTAKE_RIGHT_L3)),
  ALGAE_INTAKE_L3_LEFT(
      new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_L3, ArmState.ALGAE_INTAKE_LEFT_L3)),

  ALGAE_NET_RIGHT(new SuperstructurePosition(ElevatorState.ALGAE_NET, ArmState.ALGAE_NET_RIGHT)),
  ALGAE_NET_LEFT(new SuperstructurePosition(ElevatorState.ALGAE_NET, ArmState.ALGAE_NET_LEFT)),
  ALGAE_PROCESSOR(new SuperstructurePosition(ElevatorState.PROCESSOR, ArmState.ALGAE_PROCESSOR)),

  // LOLLIPOP WAYPOINTS
  LOLLIPOP_INTAKE(
      new SuperstructurePosition(
          ElevatorState.LOLLIPOP_CORAL_INTAKE_INTAKE, ArmState.LOLLIPOP_CORAL_INTAKE_INTAKE)),
  LOLLIPOP_INTAKE_PUSH(
      new SuperstructurePosition(
          ElevatorState.LOLLIPOP_CORAL_INTAKE_PUSH, ArmState.LOLLIPOP_CORAL_INTAKE_PUSH));

  private static final ImmutableList<Waypoint> ALL_WAYPOINTS =
      ImmutableList.copyOf(List.of(values()));

  public final SuperstructurePosition position;

  Waypoint(SuperstructurePosition position) {
    this.position = position;
  }

  public double costFor(Waypoint other) {
    return position.costFor(other.position);
  }

  public double costForLongWay(Waypoint other) {
    return position.costForLongWay(other.position);
  }

  public static void log() {
    for (var waypoint : values()) {
      DogLog.log("CollisionAvoidance/Waypoints/" + waypoint, waypoint.position.translation());
    }
    DogLog.log(
        "CollisionAvoidance/Waypoints/All",
        Arrays.stream(values())
            .map(waypoint -> waypoint.position.translation())
            .toArray(Translation2d[]::new));
  }

  /**
   * Find the closest waypoint to the given superstructure position.
   *
   * @param position The position of the superstructure.
   */
  public static Waypoint getClosest(SuperstructurePosition position) {
    if (FeatureFlags.USE_ALTERNATE_WAYPOINT_CHOOSER.getAsBoolean()) {
      var positionTranslation = position.translation();

      return Collections.min(
          ALL_WAYPOINTS,
          comparingDouble(
              waypoint -> positionTranslation.getDistance(waypoint.position.translation())));
    }

    return Collections.min(
        ALL_WAYPOINTS, comparingDouble(waypoint -> position.costFor(waypoint.position)));
  }

  public void alwaysSafe(MutableValueGraph<Waypoint, WaypointEdge> graph, Waypoint... others) {
    for (var other : others) {
      graph.putEdgeValue(
          this,
          other,
          new WaypointEdge(
              this, other, ObstructionStrategy.IGNORE_BLOCKED, ObstructionStrategy.IGNORE_BLOCKED));
    }
  }

  public void leftSideSpecial(
      MutableValueGraph<Waypoint, WaypointEdge> graph,
      ObstructionStrategy leftStrategy,
      Waypoint... others) {
    for (var other : others) {
      graph.putEdgeValue(
          this,
          other,
          new WaypointEdge(this, other, leftStrategy, ObstructionStrategy.IGNORE_BLOCKED));
    }
  }

  public void rightSideSpecial(
      MutableValueGraph<Waypoint, WaypointEdge> graph,
      ObstructionStrategy rightStrategy,
      Waypoint... others) {
    for (var other : others) {
      graph.putEdgeValue(
          this,
          other,
          new WaypointEdge(this, other, ObstructionStrategy.IGNORE_BLOCKED, rightStrategy));
    }
  }

  public void avoidClimberAlwaysSafe(
      MutableValueGraph<Waypoint, WaypointEdge> graph, Waypoint... others) {
    for (var other : others) {
      graph.putEdgeValue(
          this,
          other,
          new WaypointEdge(
                  this,
                  other,
                  ObstructionStrategy.IGNORE_BLOCKED,
                  ObstructionStrategy.IGNORE_BLOCKED)
              .avoidClimber());
    }
  }

  public void avoidClimberAlwaysSafeTeleop(
      MutableValueGraph<Waypoint, WaypointEdge> graph, Waypoint... others) {
    for (var other : others) {
      graph.putEdgeValue(
          this,
          other,
          new WaypointEdge(
                  this,
                  other,
                  ObstructionStrategy.IGNORE_BLOCKED,
                  ObstructionStrategy.IGNORE_BLOCKED)
              .avoidClimber()
              .onlyTeleop());
    }
  }

  public void avoidClimberLeftSideSpecial(
      MutableValueGraph<Waypoint, WaypointEdge> graph,
      ObstructionStrategy leftStrategy,
      Waypoint... others) {
    for (var other : others) {
      graph.putEdgeValue(
          this,
          other,
          new WaypointEdge(this, other, leftStrategy, ObstructionStrategy.IGNORE_BLOCKED)
              .avoidClimber());
    }
  }

  public void avoidClimberRightSideSpecial(
      MutableValueGraph<Waypoint, WaypointEdge> graph,
      ObstructionStrategy rightStrategy,
      Waypoint... others) {
    for (var other : others) {
      graph.putEdgeValue(
          this,
          other,
          new WaypointEdge(this, other, ObstructionStrategy.IGNORE_BLOCKED, rightStrategy)
              .avoidClimber());
    }
  }
}
