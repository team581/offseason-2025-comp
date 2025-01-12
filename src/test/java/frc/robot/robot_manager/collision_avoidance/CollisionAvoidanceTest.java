package frc.robot.robot_manager.collision_avoidance;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.ArrayList;
import org.junit.jupiter.api.Test;

public class CollisionAvoidanceTest {
  @Test
  void collidesTest() {
    var currentPose = new Translation2d(-1, 0);
    var goalPose = new Translation2d(1, 4);

    var result = CollisionAvoidance.collides(currentPose, goalPose);
    var expected = true;
    assertEquals(expected, result);
  }
  @Test
  void angleHeightToPoseTest() {
    var wristAngle = 90.0;
    var elevatorHeight = 0.0;

    var result = CollisionAvoidance.angleHeightToPose(wristAngle, elevatorHeight);
    var expected = new Translation2d(0,22);
    assertEquals(expected, result);
  }

  @Test
  void poseToSuperstructurePositionTest() {
    var currentPose = new Translation2d(0, 22);

    var result = CollisionAvoidance.poseToSuperstructurePosition(currentPose);
    var expected = new SuperstructurePosition(0.0,90.0);
    assertEquals(expected, result);
  }
  @Test
  void distancefromPosesTest() {
    var currentPose = new Translation2d(4.0, 0.0);
    var goalPose = new Translation2d(1.0, 1.0);

    var result = CollisionAvoidance.distancefromPoses(currentPose, goalPose);
    var expected = 3.1622776601683795;
    assertEquals(expected, result);
  }


  @Test
  void testPlanNoCollisionsLow() {
    SuperstructurePosition current = new SuperstructurePosition(0, 150);
    SuperstructurePosition goal = new SuperstructurePosition(1, -15);
    SuperstructurePosition expectedResult = new SuperstructurePosition(1, -15);
    var result = CollisionAvoidance.plan(current, goal);
    assertEquals(expectedResult, result);
  }

  @Test
  void testPlanNoCollisionsRight() {
    SuperstructurePosition current = new SuperstructurePosition(0, 0);
    SuperstructurePosition goal = new SuperstructurePosition(65, 80);
    SuperstructurePosition expectedResult = new SuperstructurePosition(65, 80);
    var result = CollisionAvoidance.plan(current, goal);
    assertEquals(expectedResult, result);
  }

  @Test
  void testPlanNoCollisionsHigh() {
    SuperstructurePosition current = new SuperstructurePosition(65, 10);
    SuperstructurePosition goal = new SuperstructurePosition(68, 135);
    SuperstructurePosition expectedResult = new SuperstructurePosition(68.0, 135.0);
    var result = CollisionAvoidance.plan(current, goal);
    assertEquals(expectedResult, result);
  }

  @Test
  void testPlanCollisionLowToMid() {
    SuperstructurePosition current = new SuperstructurePosition(0, 135);
    SuperstructurePosition goal = new SuperstructurePosition(20, 80);
    SuperstructurePosition expectedResult = new SuperstructurePosition(2.0, 75.0);
    var result = CollisionAvoidance.plan(current, goal);
    assertEquals(expectedResult, result);
  }
}
