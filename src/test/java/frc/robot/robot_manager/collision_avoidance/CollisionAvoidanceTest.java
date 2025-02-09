package frc.robot.robot_manager.collision_avoidance;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.robot_manager.SuperstructurePosition;
import java.util.Optional;
import org.junit.jupiter.api.Test;

public class CollisionAvoidanceTest {
  @Test
  void testInZoneGroundIntake() {
    SuperstructurePosition current = new SuperstructurePosition(0, -82.3);
    CollisionBox box = CollisionBox.BOX_3;
    var result = CollisionAvoidance.getZone(current);
    assertEquals(box, result);
  }

  @Test
  void testInZone() {
    SuperstructurePosition current = new SuperstructurePosition(0, 90);
    CollisionBox box = CollisionBox.BOX_2;
    var result = CollisionAvoidance.getZone(current);
    assertEquals(box, result);
  }

  @Test
  void testGetZoneOutofBounds() {
    SuperstructurePosition current = new SuperstructurePosition(100, 40);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_5;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone1() {
    SuperstructurePosition current = new SuperstructurePosition(25, 180);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_1;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone2() {
    SuperstructurePosition current = new SuperstructurePosition(0, 180);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_2;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone3() {
    SuperstructurePosition current = new SuperstructurePosition(0, 40);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_3;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone4() {
    SuperstructurePosition current = new SuperstructurePosition(0, 0);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_3;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone6() {
    SuperstructurePosition current = new SuperstructurePosition(67, 40);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_5;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone7() {
    SuperstructurePosition current = new SuperstructurePosition(67, 135);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_6;
    assertEquals(expectedResult, result);
  }

  @Test
  void testPlanNoCollisionsLow() {
    SuperstructurePosition current = new SuperstructurePosition(0, 180);
    SuperstructurePosition goal = new SuperstructurePosition(20, 180);
    SuperstructurePosition expectedResult = CollisionBox.BOX_1.safeZone;
    var result = CollisionAvoidance.plan(current, goal);

    assertEquals(expectedResult, result.get());
  }

  @Test
  void testPlanNoCollisionsHigh() {
    SuperstructurePosition current = new SuperstructurePosition(67, 85);
    SuperstructurePosition goal = new SuperstructurePosition(65, 180);
    var result = CollisionAvoidance.plan(current, goal);
    assertEquals(Optional.empty(), result);
  }

  @Test
  void testSkip345() {
    SuperstructurePosition current = new SuperstructurePosition(0, 40);
    SuperstructurePosition goal = new SuperstructurePosition(65, 40);
    var result = CollisionAvoidance.plan(current, goal);
    assertEquals(Optional.empty(), result);
  }

  @Test
  void testHighToLowNoSkip() {
    SuperstructurePosition current = new SuperstructurePosition(65, 50);
    SuperstructurePosition goal = new SuperstructurePosition(0, 50);
    var result = CollisionAvoidance.plan(current, goal);
    var expectedResult = CollisionBox.BOX_5.safeZone;
    assertEquals(expectedResult, result.get());
  }

  @Test
  void testStationIntakeToL4() {
    SuperstructurePosition current = new SuperstructurePosition(13, 180);
    SuperstructurePosition goal = new SuperstructurePosition(100, 40);
    var result = CollisionAvoidance.plan(current, goal);
    var expectedResult = CollisionBox.BOX_2.safeZone;
    assertEquals(expectedResult, result.get());
  }

  @Test
  void testOutToL4Unsafe() {
    SuperstructurePosition current = new SuperstructurePosition(0, 0);
    SuperstructurePosition goal = new SuperstructurePosition(100, 50);
    var result = CollisionAvoidance.plan(current, goal);
    var expectedResult = CollisionBox.BOX_4.safeZone;
    assertEquals(expectedResult, result.get());
  }

  @Test
  void testStraightUpToL4() {
    SuperstructurePosition current = new SuperstructurePosition(0, 90);
    SuperstructurePosition goal = new SuperstructurePosition(100, 50);
    var result = CollisionAvoidance.plan(current, goal);
    var expectedResult = CollisionBox.BOX_3.safeZone;
    assertEquals(expectedResult, result.get());
  }
}
