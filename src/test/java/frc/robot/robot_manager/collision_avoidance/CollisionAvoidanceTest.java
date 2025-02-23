package frc.robot.robot_manager.collision_avoidance;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.config.FeatureFlags;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.Optional;
import org.junit.jupiter.api.Test;

public class CollisionAvoidanceTest {
  @Test
  void testInZoneGroundIntake() {
    SuperstructurePosition current = new SuperstructurePosition(0, -82.3);
    CollisionBox box = CollisionBox.BOX_4;
    var result = CollisionAvoidance.getZone(current);
    assertEquals(box, result);
  }

  @Test
  void testSkipWorthyZone() {
    if (FeatureFlags.COLLISION_AVOIDANCE_BOX_SHORTCUTS.getAsBoolean()) {
      SuperstructurePosition current = new SuperstructurePosition(0, 40);
      CollisionBox box = CollisionBox.BOX_4;
      var result = CollisionAvoidance.getZone(current);
      assertEquals(box, result);
    }
  }

  @Test
  void testGetZoneOutofBounds() {
    SuperstructurePosition current = new SuperstructurePosition(100, 40);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_6;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone1() {
    SuperstructurePosition current = new SuperstructurePosition(11, 100);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_1;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone2() {
    SuperstructurePosition current = new SuperstructurePosition(0, 150);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_2;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone3() {
    SuperstructurePosition current = new SuperstructurePosition(0, 80);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_3;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone4() {
    SuperstructurePosition current = new SuperstructurePosition(0, 40);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_4;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone5() {
    SuperstructurePosition current = new SuperstructurePosition(30, 40);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_5;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone6() {
    SuperstructurePosition current = new SuperstructurePosition(54, 40);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_6;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone7() {
    SuperstructurePosition current = new SuperstructurePosition(54, 135);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_7;
    assertEquals(expectedResult, result);
  }

  @Test
  void testPlanNoCollisionsLow() {
    SuperstructurePosition current = new SuperstructurePosition(0, 120);
    SuperstructurePosition goal = new SuperstructurePosition(20, 120);
    SuperstructurePosition expectedResult = CollisionBox.BOX_1.safeZone;
    var result = CollisionAvoidance.plan(current, goal);

    assertEquals(expectedResult, result.get());
  }

  @Test
  void testPlanNoCollisionsHigh() {
    SuperstructurePosition current = new SuperstructurePosition(54, 85);
    SuperstructurePosition goal = new SuperstructurePosition(55, 135);
    var result = CollisionAvoidance.plan(current, goal);
    assertEquals(Optional.empty(), result);
  }

  @Test
  void testPlanBackwardsHigh() {
    SuperstructurePosition current = new SuperstructurePosition(54, 40);
    SuperstructurePosition goal = new SuperstructurePosition(56, 135);
    var expectedResult = CollisionBox.BOX_7.safeZone;
    var result = CollisionAvoidance.plan(current, goal);
    assertEquals(expectedResult, result.get());
  }

  @Test
  void testSkip345() {
    if (FeatureFlags.COLLISION_AVOIDANCE_BOX_SHORTCUTS.getAsBoolean()) {
      SuperstructurePosition current = new SuperstructurePosition(0, 40);
      SuperstructurePosition goal = new SuperstructurePosition(54, 40);
      var result = CollisionAvoidance.plan(current, goal);
      assertEquals(Optional.empty(), result);
    }
  }

  @Test
  void testHighToLowNoSkip() {
    SuperstructurePosition current = new SuperstructurePosition(54, 80);
    SuperstructurePosition goal = new SuperstructurePosition(0, 50);
    var result = CollisionAvoidance.plan(current, goal);
    var expectedResult = CollisionBox.BOX_6.safeZone;
    assertEquals(expectedResult, result.get());
  }

  @Test
  void testStationIntakeToL4() {
    SuperstructurePosition current = new SuperstructurePosition(13, 120);
    SuperstructurePosition goal = new SuperstructurePosition(100, 40);
    var result = CollisionAvoidance.plan(current, goal);
    var expectedResult = CollisionBox.BOX_2.safeZone;
    assertEquals(expectedResult, result.get());
  }

  @Test
  void testBottomToL4Unsafe() {
    SuperstructurePosition current = new SuperstructurePosition(0, 0);
    SuperstructurePosition goal = new SuperstructurePosition(100, 50);
    var result = CollisionAvoidance.plan(current, goal);
    var expectedResult = CollisionBox.BOX_5.safeZone;
    assertEquals(expectedResult, result.get());
  }

  @Test
  void testStraightUpToL4() {
    SuperstructurePosition current = new SuperstructurePosition(0, 80);
    SuperstructurePosition goal = new SuperstructurePosition(100, 50);
    var result = CollisionAvoidance.plan(current, goal);
    var expectedResult = CollisionBox.BOX_4.safeZone;
    assertEquals(expectedResult, result.get());
  }

  @Test
  void shortCutPossibleElevator0NoShortcut() {
    SuperstructurePosition current = new SuperstructurePosition(15, 120);
    SuperstructurePosition goal = new SuperstructurePosition(0, -40);
    var result = CollisionAvoidance.plan(current, goal);
    var expectedResult = CollisionBox.BOX_2.safeZone;
    assertEquals(expectedResult, result.get());
  }

  @Test
  void shortCutPossibleElevator0YesShortcut() {
    SuperstructurePosition current = new SuperstructurePosition(0, 120);
    SuperstructurePosition goal = new SuperstructurePosition(0, -40);
    var result = CollisionAvoidance.plan(current, goal);
    var expectedResult = Optional.empty();
    assertEquals(expectedResult, result);
  }

  @Test
  void stationIntakeToStow() {
    SuperstructurePosition current = new SuperstructurePosition(10.5, 120);
    SuperstructurePosition goal = new SuperstructurePosition(0, 33);
    var result = CollisionAvoidance.plan(current, goal);
    var expectedResult = CollisionBox.BOX_2.safeZone;
    assertEquals(expectedResult, result.get());
  }
}
