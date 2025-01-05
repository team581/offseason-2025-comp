package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class ControllerHelpers {
  public static double getDeadbanded(double joystickVal, double threshold) {
    double newJoystickVal = joystickVal;
    if (Math.abs(joystickVal) < threshold) {
      newJoystickVal = 0;
    } else {
      newJoystickVal =
          getSign(joystickVal) * (1 / (1 - threshold)) * (Math.abs(joystickVal) - threshold);
    }
    return newJoystickVal;
  }

  private static double getSign(double num) {
    return num >= 0 ? 1 : -1;
  }

  public static double getExponent(double joystickVal, double exponent) {
    return getSign(joystickVal) * Math.abs(Math.pow(Math.abs(joystickVal), exponent));
  }

  public static Translation2d fromCircularDiscCoordinates(double x, double y) {
    // https://stackoverflow.com/a/32391780
    var mappedX =
        0.5 * goodSquareRoot(2 + Math.pow(x, 2) - Math.pow(y, 2) + 2 * x * Math.sqrt(2))
            - 0.5 * goodSquareRoot(2 + Math.pow(x, 2) - Math.pow(y, 2) - 2 * x * Math.sqrt(2));
    var mappedY =
        0.5 * goodSquareRoot(2 - Math.pow(x, 2) + Math.pow(y, 2) + 2 * y * Math.sqrt(2))
            - 0.5 * goodSquareRoot(2 - Math.pow(x, 2) + Math.pow(y, 2) - 2 * y * Math.sqrt(2));

    return desaturate(mappedX, mappedY);
  }

  private static double goodSquareRoot(double value) {
    return Math.copySign(Math.sqrt(Math.abs(value)), value);
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
}
