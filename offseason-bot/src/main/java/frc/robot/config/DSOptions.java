package frc.robot.config;

import com.team581.config.DSOption;
import edu.wpi.first.networktables.BooleanSubscriber;

public final class DSOptions {
  public static final BooleanSubscriber SENSOR_BROKEN = DSOption.of("SensorBroken", false);

  private DSOptions() {}
}
