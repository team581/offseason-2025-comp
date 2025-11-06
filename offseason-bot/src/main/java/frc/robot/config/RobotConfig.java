package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.team581.config.CameraConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;

public record RobotConfig(
    IntakeConfig intake,
    ClawConfig claw,
    DeployConfig deploy,
    ClimberConfig climber,
    SingulatorConfig singulator,
    ElevatorConfig elevator,
    ArmConfig arm,
    VisionConfig vision,
    LightsConfig lights,
    SwerveConfig swerve) {
  public record IntakeConfig(
      String canBusName, int motorId, Debouncer debouncer, TalonFXConfiguration motorConfig) {}

  public record ClawConfig(
      String canBusName,
      int motorId,
      int candiId,
      boolean sensorFlipped,
      Debouncer debouncer,
      TalonFXConfiguration motorConfig) {}

  public record SingulatorConfig(
      String canBusName,
      int leftMotorId,
      int rightMotorId,
      int topSensorPortId,
      int bottonSensorPortId,
      Debouncer topDebouncer,
      Debouncer bottonDebouncer,
      TalonFXConfiguration leftMotorConfig,
      TalonFXConfiguration rightMotorConfig) {}

  public record DeployConfig(
      String canBusName,
      int motorId,
      double minAngle,
      double maxAngle,
      double homingVoltage,
      double homingCurrentThreshold,
      double homingEndPosition,
      TalonFXConfiguration motorConfig) {}

  public record ClimberConfig(
      String canBusName,
      int climbMotorId,
      int cancoderId,
      double minAngle,
      double maxAngle,
      TalonFXConfiguration climbMotorConfig,
      CANcoderConfiguration cancoderConfig) {}

  public record ElevatorConfig(
      String canBusName,
      int motorId,
      double homingVoltage,
      double homingCurrentThreshold,
      double homingEndHeight,
      double maxHeight,
      double minHeight,
      TalonFXConfiguration motorConfig) {}

  public record ArmConfig(
      String canBusName,
      int motorId,
      double minAngle,
      double maxAngle,
      TalonFXConfiguration motorConfig,
      double homingPosition) {}

  public record VisionConfig(
      double xyStdDev,
      double thetaStdDev,
      CameraConfig leftLimelightConfig,
      CameraConfig rightLimelightConfig,
      CameraConfig gpCameraConfig,
      Pose3d gamePieceToRobotPose) {}

  public record LightsConfig(String canBusName, int candleId) {}

  public record SwerveConfig(
      PhoenixPIDController snapController,
      boolean invertRotation,
      boolean invertX,
      boolean invertY) {}

  public static RobotConfig get() {
    return CompConfig.COMPETITION_BOT;
  }
}
