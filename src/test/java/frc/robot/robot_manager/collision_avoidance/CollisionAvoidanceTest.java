package frc.robot.robot_manager.collision_avoidance;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.robot_manager.SuperstructurePosition;
import org.junit.jupiter.api.Test;

public class CollisionAvoidanceTest {
  @Test
  public void getClosestNodeStowedTest() {
    var result = Waypoint.getClosest(new SuperstructurePosition(43, -90));
    Waypoint expected = Waypoint.HANDOFF;

    assertEquals(expected, result);
  }
}
