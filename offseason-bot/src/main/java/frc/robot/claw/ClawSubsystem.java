package frc.robot.claw;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S1StateValue;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.DSOptions;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class ClawSubsystem extends StateMachineSubsystem<ClawState> {
  private final TalonFX motor;
  private final CANdi candi;
  private final Debouncer debouncer = RobotConfig.get().claw().debouncer();

  private boolean sensorRaw = false;
  private boolean sensorDebounced = false;

  public ClawSubsystem(TalonFX motor, CANdi candi) {
    super(SubsystemPriority.INTAKE, ClawState.IDLE_NO_GP);

    motor.getConfigurator().apply(RobotConfig.get().claw().motorConfig());
    this.motor = motor;
    this.candi = candi;
  }

  @Override
  protected void collectInputs() {
    var candiValue = candi.getS1State().getValue();
    sensorRaw =
        RobotConfig.get().claw().sensorFlipped()
            ? candiValue == S1StateValue.Low
            : candiValue != S1StateValue.Low;

    if (RobotBase.isSimulation() || DSOptions.SENSOR_BROKEN.getAsBoolean()) {
      sensorRaw =
          switch (getState()) {
            case CORAL_HANDOFF -> timeout(0.5);
            case IDLE_NO_GP -> false;
            case IDLE_W_ALGAE, IDLE_W_CORAL -> true;
            case INTAKING_ALGAE -> timeout(1.5);
            case SCORE_CORAL -> !timeout(0.2);
            case SCORE_CORAL_L1 -> !timeout(0.4);
            case SCORE_ALGAE_NET, SCORE_ALGAE_PROCESSOR, OUTTAKING -> !timeout(0.25);
          };
    }

    sensorDebounced = debouncer.calculate(sensorRaw);
  }

  public boolean getHasGP() {
    return sensorDebounced;
  }

  public void setState(ClawState newState) {
    setStateFromRequest(newState);
  }

  @Override
  protected void afterTransition(ClawState newState) {
    switch (newState) {
      case IDLE_NO_GP -> {
        motor.disable();
      }
      case IDLE_W_ALGAE -> {
        motor.setVoltage(12.0);
      }
      case IDLE_W_CORAL -> {
        motor.setVoltage(1.2);
      }
      case INTAKING_ALGAE -> {
        motor.setVoltage(12.0);
      }
      case CORAL_HANDOFF -> {
        motor.setVoltage(12.0);
      }
      case SCORE_ALGAE_NET -> {
        motor.setVoltage(-12);
      }
      case SCORE_ALGAE_PROCESSOR -> {
        motor.setVoltage(-5);
      }
      case SCORE_CORAL -> {
        motor.setVoltage(-5);
      }
      case SCORE_CORAL_L1 -> {
        motor.setVoltage(-1.5);
      }
      case OUTTAKING -> {
        motor.setVoltage(-6);
      }
    }
  }

  @Override
  public void whileInState(ClawState currentState) {
    DogLog.log("Claw/Motor/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble(), Volts);
    DogLog.log("Claw/Motor/StatorCurrent", motor.getStatorCurrent().getValueAsDouble(), Amps);
    DogLog.log("Claw/Motor/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble(), Amps);
    DogLog.log("Claw/Sensors/SensorRaw", sensorRaw);
    DogLog.log("Claw/Sensors/SensorDebounced", sensorDebounced);
  }
}
