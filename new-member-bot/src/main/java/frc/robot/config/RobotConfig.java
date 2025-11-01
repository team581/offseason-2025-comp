package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.team581.config.CameraConfig;
import com.team581.mechanisms.VelocityDetector;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Velocity;

public record RobotConfig(
    ClawConfig claw,
    ClimberConfig climber,
    ElevatorConfig elevator,
    WristConfig wrist,
    VisionConfig vision,
    SwerveConfig swerve) {
  public record ClawConfig(
      String canBusName,
      int motorId,
      double coralMaxVelocity,
      VelocityDetector coralDetector,
      double algaeMaxVelocity,
      VelocityDetector algaeDetector,
      TalonFXConfiguration motorConfig) {}

  public record ClimberConfig(
      String canBusName,
      int climbMotorId,
      int cancoderId,
      int grabMotorId,
      int canrangeId,
      double minAngle,
      double maxAngle,
      TalonFXConfiguration climbMotorConfig,
      CANcoderConfiguration cancoderConfig,
      TalonFXConfiguration grabMotorConfig,
      CANrangeConfiguration canRangeConfig) {}

  public record ElevatorConfig(
      String canBusName,
      int motorId,
      double homingVoltage,
      double homingCurrentThreshold,
      double homingEndHeight,
      double maxHeight,
      double minHeight,
      TalonFXConfiguration motorConfig) {}

  public record WristConfig(
      String canBusName,
      int motorId,
      double rangeOfMotionDeg,
      TalonFXConfiguration motorConfig,
      double homingPosition) {}

  public record VisionConfig(
      double xyStdDev,
      double thetaStdDev,
      Pose3d robotPoseRelativeToCalibration,
      CameraConfig mainLimelightConfig) {}

  public record SwerveConfig(
      PhoenixPIDController snapController,
      boolean invertRotation,
      boolean invertX,
      boolean invertY) {}

  public static RobotConfig get() {
    return CompConfig.COMPETITION_BOT;
  }
}
