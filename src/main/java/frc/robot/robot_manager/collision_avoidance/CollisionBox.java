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
          new Translation2d(-20, 12), new Translation2d(-14, 30)),
      new SuperstructurePosition(13, 180)), // whatever it is for station intake
  BOX_2(
      2,
      new Rectangle2d( // bottom backwards
          new Translation2d(-20, -20), new Translation2d(7.5, 30)),
      new SuperstructurePosition(0, 180)), // 0 and angle of station intake
  BOX_3(
      3,
      new Rectangle2d( // forwards bottom
          new Translation2d(7.5, -20), new Translation2d(20, 25)),
      new SuperstructurePosition(0, 45)),

  BOX_4(
      4,
      new Rectangle2d( // forwards middle
          new Translation2d(7.5, 25), new Translation2d(20, 40)),
      new SuperstructurePosition(30, 45)),
  BOX_5(
      5,
      new Rectangle2d( // forwards top
          new Translation2d(7.5, 40), new Translation2d(20, 86)),
      new SuperstructurePosition(67, 45)),
  BOX_6(
      6,
      new Rectangle2d( // forwards Top
          new Translation2d(-20, 56.5), new Translation2d(7.5, 86)),
      new SuperstructurePosition(67, 135)),
      ;

  public final int id;
  public final Rectangle2d bounds;
  public final SuperstructurePosition safeZone;

  public boolean shortCutPossible(CollisionBox goal) {
    if(this==goal){
      return false;
    }
    return switch(goal) {
      case BOX_3, BOX_4, BOX_5-> switch(this) {
        case BOX_3, BOX_4, BOX_5 -> true;
        default -> false;
      };
      default -> false;
    };
  }

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
