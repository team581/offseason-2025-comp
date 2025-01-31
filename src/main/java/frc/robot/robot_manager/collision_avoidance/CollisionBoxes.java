package frc.robot.robot_manager.collision_avoidance;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.robot_manager.BoxZone;
import frc.robot.robot_manager.SuperstructurePosition;

public enum CollisionBoxes {
  BOX_1(
      new BoxZone( // zone where the station intake happens
          new Translation2d(-20, 14),
          new Translation2d(-14, 30),
          new SuperstructurePosition(20, 180))), // whatever it is for station intake
  BOX_2(
      new BoxZone( // bottom right zone
          new Translation2d(-20, 0),
          new Translation2d(-1, 30),
          new SuperstructurePosition(0, 180))), // 0 and angle of station intake
  BOX_3(
      new BoxZone( // middle to the left
          new Translation2d(-1, 0), new Translation2d(20, 20), new SuperstructurePosition(0, 45))),

  BOX_4(
      new BoxZone( // middle left zone
          new Translation2d(1, 20), new Translation2d(20, 45), new SuperstructurePosition(30, 45))),
  BOX_5(
      new BoxZone( // top left zone
          new Translation2d(0, 45), new Translation2d(20, 86), new SuperstructurePosition(67, 45))),
  BOX_6(
      new BoxZone( // top right zone
          new Translation2d(-20, 67),
          new Translation2d(0, 86),
          new SuperstructurePosition(67, 135)));
  public final BoxZone box;

  private CollisionBoxes(BoxZone box) {
    this.box = box;
  }
}
