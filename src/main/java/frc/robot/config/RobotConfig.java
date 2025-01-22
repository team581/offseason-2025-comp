package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.vision.interpolation.InterpolatedVisionDataset;

public record RobotConfig(
    String robotName,
    ElevatorConfig elevator,
    IntakeConfig intake,
    SwerveConfig swerve,
    VisionConfig vision,
    WristConfig wrist,
    PivotConfig pivot,
    ClimberConfig climber,
    LightsConfig lights) {
  public record ElevatorConfig(
      String canBusName,
      int leftMotorID,
      int rightMotorID,
      TalonFXConfiguration leftMotorConfig,
      TalonFXConfiguration rightMotorConfig,
      double homingEndHeight,
      double minHeight,
      double maxHeight,
      double tolerance) {}

  public record IntakeConfig(
      String canBusName,
      int topMotorID,
      int bottomMotorID,
      Debouncer topDebouncer,
      Debouncer bottomDebouncer,
      TalonFXConfiguration topMotorConfig,
      TalonFXConfiguration bottomMotorConfig) {}

  public record SwerveConfig(
      PhoenixPIDController snapController,
      boolean invertRotation,
      boolean invertX,
      boolean invertY,
      TalonFXConfiguration driveMotorConfig,
      TalonFXConfiguration steerMotorConfig) {}

  public record VisionConfig(
      int translationHistoryArraySize,
      double xyStdDev,
      double thetaStdDev,
      InterpolatedVisionDataset interpolatedVisionSet) {}

  public record WristConfig(
      String canBusName,
      int motorID,
      TalonFXConfiguration motorConfig,
      double minAngle,
      double maxAngle) {}

  public record ClimberConfig(
      String canBusName,
      int motorID,
      int cancoderID,
      TalonFXConfiguration motorConfig,
      CANcoderConfiguration cancoderConfig) {}

  public record PivotConfig(
      String canBusName,
      int motorID,
      TalonFXConfiguration motorConfig,
      double homingCurrentThreshold,
      double homingPosition) {}

  public record LightsConfig(String canBusName, int candleID) {}

  // TODO: Change this to false during events
  public static final boolean IS_DEVELOPMENT = true;
  public static final String SERIAL_NUMBER = System.getenv("serialnum");

  public static RobotConfig get() {
    return PracticeConfig.practiceBot;
  }
}
