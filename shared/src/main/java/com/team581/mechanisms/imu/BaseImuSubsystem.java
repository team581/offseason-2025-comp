package com.team581.mechanisms.imu;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team581.util.scheduling.SubsystemPriorityBase;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;

public class BaseImuSubsystem extends StateMachineSubsystem<ImuState> {
  protected final SwerveDrivetrain<?, ?, ?> drivetrain;

  protected SwerveDriveState driveState = new SwerveDriveState();
  protected double robotHeading = 0;
  protected double robotAngularVelocity = 0;

  public BaseImuSubsystem(SubsystemPriorityBase priority, SwerveDrivetrain<?, ?, ?> drivetrain) {
    super(priority, ImuState.DEFAULT_STATE);

    this.drivetrain = drivetrain;
  }

  @Override
  protected void collectInputs() {
    driveState = drivetrain.getState();
    robotHeading = MathUtil.inputModulus(driveState.Pose.getRotation().getDegrees(), -180, 180);
    robotAngularVelocity = Math.toDegrees(driveState.Speeds.omegaRadiansPerSecond);
  }

  public double getRobotHeading() {
    return robotHeading;
  }

  public double getRobotAngularVelocity() {
    return robotAngularVelocity;
  }

  public void setAngle(double zeroAngle) {
    drivetrain.getPigeon2().setYaw(zeroAngle);
  }

  @Override
  public void whileInState(ImuState currentState) {
    DogLog.log("Imu/RobotHeading", robotHeading, Degrees);
    DogLog.log("Imu/AngularVelocity", robotAngularVelocity, DegreesPerSecond);
  }
}
