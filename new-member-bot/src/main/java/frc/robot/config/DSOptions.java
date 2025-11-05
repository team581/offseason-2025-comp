package frc.robot.config;

import com.team581.config.DSOption;
import edu.wpi.first.networktables.BooleanSubscriber;

public final class DSOptions {
  public static final BooleanSubscriber PRACTICE_MODE = DSOption.of("PracticeMode", false);

  private DSOptions() {}
}
