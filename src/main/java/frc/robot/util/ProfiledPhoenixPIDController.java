package frc.robot.util;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.MathUtil;

public class ProfiledPhoenixPIDController extends PhoenixPIDController {
  private double maxOutput;

  /**
   * Allocates a PIDController with the given constants for kp, ki, and kd.
   *
   * @param kp The proportional coefficient.
   * @param ki The integral coefficient.
   * @param kd The derivative coefficient.
   * @param maxOutput The maximum output of the PID controller.
   */
  public ProfiledPhoenixPIDController(double kp, double ki, double kd, double maxOutput) {
    super(kp, ki, kd);
    this.maxOutput = Math.abs(maxOutput);
  }

  @Override
  public double calculate(double measurement, double setpoint, double currentTimestamp) {
    var output = super.calculate(measurement, setpoint, currentTimestamp);

    return MathUtil.clamp(output, -maxOutput, maxOutput);
  }

  /**
   * Sets the maximum output of the PID controller.
   *
   * @param maxOutput The maximum output of the PID controller.
   */
  public void setMaxOutput(double maxOutput) {
    this.maxOutput = maxOutput;
  }
}
