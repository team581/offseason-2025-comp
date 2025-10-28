package frc.robot.imu;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.team581.mechanisms.imu.BaseImuSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.util.scheduling.SubsystemPriority;

public class ImuSubsystem extends BaseImuSubsystem {
  private static final double IS_TILTED_THRESHOLD = 4.0;
  private final Debouncer isTiltedDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private double pitch;
  private double roll;

  public ImuSubsystem(SwerveDrivetrain<?, ?, ?> drivetrain) {
    super(SubsystemPriority.IMU, drivetrain);
  }

  @Override
  protected void collectInputs() {
    super.collectInputs();

    pitch = drivetrain.getPigeon2().getPitch().getValueAsDouble();
    roll = drivetrain.getPigeon2().getRoll().getValueAsDouble();
  }

  public boolean isFlatDebounced() {
    return isTiltedDebouncer.calculate(
        MathUtil.isNear(pitch, 0, IS_TILTED_THRESHOLD,-90,90)
            && MathUtil.isNear(roll, 0, IS_TILTED_THRESHOLD, -180,180));
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("Imu/Roll", roll);
    DogLog.log("Imu/Pitch", pitch);
  }
}
