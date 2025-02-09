package frc.robot.imu;

import com.ctre.phoenix6.hardware.Pigeon2;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ImuSubsystem extends StateMachine<ImuState> {
  private final Pigeon2 imu;
  private double robotHeading = 0;
  private double pitch;
  private double angularVelocity;
  private double pitchRate;
  private double roll;
  private double rollRate;

  public ImuSubsystem(Pigeon2 imu) {
    super(SubsystemPriority.IMU, ImuState.DEFAULT_STATE);
    this.imu = imu;
  }

  @Override
  protected void collectInputs() {
    robotHeading = MathUtil.inputModulus(imu.getYaw().getValueAsDouble(), -180, 180);
    angularVelocity = imu.getAngularVelocityZWorld().getValueAsDouble();
    pitch = imu.getPitch().getValueAsDouble();
    pitchRate = imu.getAngularVelocityYWorld().getValueAsDouble();
    roll = imu.getRoll().getValueAsDouble();
    rollRate = imu.getAngularVelocityXWorld().getValueAsDouble();
  }

  public double getRobotHeading() {
    return robotHeading;
  }

  public double getRobotAngularVelocity() {
    return angularVelocity;
  }

  public double getPitch() {
    return pitch;
  }

  public double getPitchRate() {
    return pitchRate;
  }

  public double getRoll() {
    return roll;
  }

  public double getRollRate() {
    return rollRate;
  }

  public void setAngle(double zeroAngle) {
    this.imu.setYaw(zeroAngle);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Imu/RobotHeading", robotHeading);
    DogLog.log("Imu/AngularVelocity", angularVelocity);
  }
}
