package frc.robot.robot_manager.collision_avoidance;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.robot_manager.SuperstructurePosition;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

public class CollisionAvoidanceTest {
  @Test
  public void stowedUpToUpRightAstarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 90),
            new SuperstructurePosition(50, 0),
            ObstructionKind.NONE);
    var expected = List.of(Waypoint.STOWED_UP, Waypoint.L3_RIGHT);

    assertEquals(expected, new ArrayList<>(result.orElseThrow()));
  }

  @Test
  public void lowLeftToStowedAstarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 180),
            new SuperstructurePosition(50, -90),
            ObstructionKind.NONE);
    // Deque<Waypoint> expected = new ArrayDeque<Waypoint>();
    // expected.add(Waypoint.LOLLIPOP_INTAKE_LEFT);
    // expected.add(Waypoint.L3_LEFT);
    // expected.add(Waypoint.HANDOFF);
    var expected = List.of(Waypoint.LOLLIPOP_INTAKE_LEFT, Waypoint.L3_LEFT, Waypoint.HANDOFF);

    assertEquals(expected, new ArrayList<>(result.orElseThrow()));
  }
  @Test
  public void lowRightToStowedAstarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 0),
            new SuperstructurePosition(50, -90),
            ObstructionKind.NONE);
    // Deque<Waypoint> expected = new ArrayDeque<Waypoint>();
    // expected.add(Waypoint.LOLLIPOP_INTAKE_LEFT);
    // expected.add(Waypoint.L3_LEFT);
    // expected.add(Waypoint.HANDOFF);
    var expected = List.of(Waypoint.L1_RIGHT, Waypoint.L3_RIGHT, Waypoint.HANDOFF);

    assertEquals(expected, new ArrayList<>(result.orElseThrow()));
  }

  @Test
  public void alreadyThereTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 180),
            new SuperstructurePosition(0, 180),
            ObstructionKind.NONE);
    var expected = List.of(Waypoint.LOLLIPOP_INTAKE_LEFT);
    assertEquals(expected, new ArrayList<>(result.orElseThrow()));
  }

  @Test
  public void getClosestNodeTest() {
    var result = Waypoint.getClosest(new SuperstructurePosition(0, 180));
    Waypoint expected = Waypoint.LOLLIPOP_INTAKE_LEFT;

    assertEquals(expected, result);
  }

  @Test
  public void getClosestNodeStowedTest() {
    var result = Waypoint.getClosest(new SuperstructurePosition(50, -90));
    Waypoint expected = Waypoint.HANDOFF;

    assertEquals(expected, result);
  }
}
