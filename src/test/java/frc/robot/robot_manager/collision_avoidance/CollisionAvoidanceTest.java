package frc.robot.robot_manager.collision_avoidance;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.robot_manager.SuperstructurePosition;
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
    var expected = List.of(Waypoint.STOWED_UP, Waypoint.L4_RIGHT);

    assertEquals(expected, result.get());
  }

  @Test
  public void lowLeftToStowedAstarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 180),
            new SuperstructurePosition(50, -90),
            ObstructionKind.NONE);
    var expected = List.of(Waypoint.LOLLIPOP_INTAKE_LEFT, Waypoint.L3_LEFT, Waypoint.HANDOFF);

    assertEquals(expected, result.get());
  }

  @Test
  public void alreadyThereTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 180),
            new SuperstructurePosition(0, 180),
            ObstructionKind.NONE);
    assertEquals(List.of(Waypoint.LOLLIPOP_INTAKE_LEFT), result.orElseThrow());
  }

  @Test
  public void getClosestNodeTest() {
    var result = Waypoint.getClosest(new SuperstructurePosition(0, 180));
    Waypoint expected = Waypoint.LOLLIPOP_INTAKE_LEFT;

    assertEquals(expected, result);
  }
}
