package frc.robot.autos.trailblazer.trackers.pure_pursuit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PurePursuitUtils {
  public static double add(double a, double b) {
    return a + b;
  }

  public static Pose2d getPerpendicularPoint(Pose2d startPoint, Pose2d endPoint, Pose2d robotPose) {
    var x1 = startPoint.getX();
    var y1 = startPoint.getY();

    var x2 = endPoint.getX();
    var y2 = endPoint.getY();

    var x3 = robotPose.getX();
    var y3 = robotPose.getY();

    if (y1 == y2) {
      return new Pose2d(x3, y1, new Rotation2d());
    }
    if (x1 == x2) {
      return new Pose2d(x1, y3, new Rotation2d());
    }
    // Find the slope of the path and y-int
    double pathSlope = (y2 - y1) / (x2 - x1);
    double yInt = y1 - pathSlope * x1;

    // Find the slope and y-int of the perpendicular line
    double perpSlope = -1 / pathSlope;
    double perpYInt = y3 - perpSlope * x3;

    // Calculate the perpendicular intersection
    double perpX = (perpYInt - yInt) / (pathSlope - perpSlope);
    double perpY = pathSlope * perpX + yInt;

    return new Pose2d(perpX, perpY, new Rotation2d());
  }

  public static Pose2d getLookaheadPoint(
      Pose2d startPoint, Pose2d endPoint, Pose2d pointOnPath, double lookaheadDistance) {
    var x1 = startPoint.getX();
    var y1 = startPoint.getY();

    var x2 = endPoint.getX();
    var y2 = endPoint.getY();

    var x = pointOnPath.getX();
    var y = pointOnPath.getY();

    var xLookahead =
        x
            + lookaheadDistance
                * ((x2 - x1) / (Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2))));
    var yLookahead =
        y
            + lookaheadDistance
                * ((y2 - y1) / (Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2))));
    return new Pose2d(xLookahead, yLookahead, new Rotation2d());
  }

  private PurePursuitUtils() {}
}
