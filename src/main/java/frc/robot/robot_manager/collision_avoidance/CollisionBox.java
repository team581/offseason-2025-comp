package frc.robot.robot_manager.collision_avoidance;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.ArrayList;
import java.util.List;

public enum CollisionBox {
  BOX_1(
      1,
      new Rectangle2d( // zone where the station intake happens
          new Translation2d(-20, 14), new Translation2d(-14, 30)),
      new SuperstructurePosition(20, 180)), // whatever it is for station intake
  BOX_2(
      2,
      new Rectangle2d( // bottom right zone
          new Translation2d(-20, -20), new Translation2d(-1, 30)),
      new SuperstructurePosition(0, 180)), // 0 and angle of station intake
  BOX_3(
      3,
      new Rectangle2d( // middle to the left
          new Translation2d(-1, -20), new Translation2d(20, 23)),
      new SuperstructurePosition(0, 45)),

  BOX_4(
      4,
      new Rectangle2d( // middle left zone
          new Translation2d(1, 23), new Translation2d(20, 45)),
      new SuperstructurePosition(30, 45)),
  BOX_5(
      5,
      new Rectangle2d( // top left zone
          new Translation2d(0, 45), new Translation2d(20, 86)),
      new SuperstructurePosition(67, 45)),
  BOX_6(
      6,
      new Rectangle2d( // top right zone
          new Translation2d(-20, 67), new Translation2d(0, 86)),
      new SuperstructurePosition(67, 135));

  public final int id;
  public final Rectangle2d bounds;
  public final SuperstructurePosition safeZone;

  public static CollisionBox getById(int id) {
    return switch (id) {
      case 1 -> BOX_1;
      case 2 -> BOX_2;
      case 3 -> BOX_3;
      case 4 -> BOX_4;
      case 5 -> BOX_5;
      case 6 -> BOX_6;
      default -> BOX_3;
    };
  }

  public static void visualize() {
    var allCorners = new ArrayList<List<Translation2d>>(CollisionBox.values().length);

    for (var box : values()) {
      var corners = getCorners(box.bounds);
      allCorners.add(corners);
      DogLog.log(
          "CollisionAvoidance/Boxes/" + box.toString(), corners.toArray(Translation2d[]::new));
    }

    DogLog.log(
        "CollisionAvoidance/Boxes/All",
        allCorners.stream().flatMap(List::stream).toArray(Translation2d[]::new));
  }

  private static List<Translation2d> getCorners(Rectangle2d rectangle) {
    return List.of(
        new Translation2d(
            rectangle.getCenter().getX() - rectangle.getXWidth() / 2,
            rectangle.getCenter().getY() - rectangle.getYWidth() / 2),
        new Translation2d(
            rectangle.getCenter().getX() + rectangle.getXWidth() / 2,
            rectangle.getCenter().getY() - rectangle.getYWidth() / 2),
        new Translation2d(
            rectangle.getCenter().getX() + rectangle.getXWidth() / 2,
            rectangle.getCenter().getY() + rectangle.getYWidth() / 2),
        new Translation2d(
            rectangle.getCenter().getX() - rectangle.getXWidth() / 2,
            rectangle.getCenter().getY() + rectangle.getYWidth() / 2));
  }

  private CollisionBox(int id, Rectangle2d bounds, SuperstructurePosition safeZone) {
    this.id = id;
    this.bounds = bounds;
    this.safeZone = safeZone;
  }
}
