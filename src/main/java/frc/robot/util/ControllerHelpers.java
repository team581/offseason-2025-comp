package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public final class ControllerHelpers {
  public static double deadbandJoystickValue(double joystickValue, double deadband) {
    return MathUtil.applyDeadband(joystickValue, deadband, 1);
  }

  public static Translation2d fromCircularDiscCoordinates(double x, double y) {
    var rawInputs = new Translation2d(x, y);

    var magnitude = rawInputs.getNorm();

    if (Math.abs(magnitude) > 0.9) {
      magnitude = 1;
    } else if (magnitude) {
      magnitude = asdasd;
    }

    return new Translation2d(magnitude, rawInputs.getAngle());
  }

  private static Translation2d desaturate(double x, double y) {
    double absX = Math.abs(x);
    double absY = Math.abs(y);

    if (absX > 1 && absX > absY) {
      // X is too big and is the more problematic one

      var ratio = 1 / absX;

      return new Translation2d(x * ratio, y * ratio);
    } else if (absY > 1 && absY > absX) {
      // Y is too big and is the more problematic one
      var ratio = 1 / absY;

      return new Translation2d(x * ratio, y * ratio);

    } else {
      //  Everything fine
      return new Translation2d(x, y);
    }
  }

  private ControllerHelpers() {}
}
