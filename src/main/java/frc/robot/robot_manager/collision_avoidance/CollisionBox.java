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
          new Translation2d(-24, 17.0), new Translation2d(-20, 25.0)),
      new SuperstructurePosition(11, 120)), // whatever it is for station intake
  BOX_2(
      2,
      new Rectangle2d( // bottom backwards
          new Translation2d(-24, -4.0), new Translation2d(-10, 25.0)),
      new SuperstructurePosition(0, 120)), // 0 and angle of station intake
  BOX_3(
      3,
      new Rectangle2d( // middle
          new Translation2d(-10, -4.0), new Translation2d(6, 25.0)),
      new SuperstructurePosition(0, 65)),
  BOX_4(
      4,
      new Rectangle2d( // forwards bottom
          new Translation2d(6, -11), new Translation2d(24, 27)),
      new SuperstructurePosition(0, 30)),
  BOX_5(
      5,
      new Rectangle2d( // forwards middle
          new Translation2d(6, 27), new Translation2d(24, 60)),
      new SuperstructurePosition(30, 40)),
  BOX_6(
      6,
      new Rectangle2d( // forwards top
          new Translation2d(6, 60), new Translation2d(24, 80)),
      new SuperstructurePosition(56, 40)),
  BOX_7(
      7,
      new Rectangle2d( // forwards Top
          new Translation2d(-24, 54), new Translation2d(6, 80)),
      new SuperstructurePosition(56, 135));

  public static final boolean BOX_SHORTCUTS_ENABLED = false;

  public final int id;
  public final Rectangle2d bounds;
  public final SuperstructurePosition safeZone;

  public boolean shortCutPossible(CollisionBox goal) {
    if (!BOX_SHORTCUTS_ENABLED) {
      return false;
    }

    if (this == goal) {
      return false;
    }
    return switch (goal) {
      case BOX_4, BOX_5, BOX_6 ->
          switch (this) {
            case BOX_4, BOX_5, BOX_6 -> true;
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
      case 7 -> BOX_7;
      default -> BOX_4;
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
