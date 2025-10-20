package frc.robot.claw;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S1StateValue;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class ClawSubsystem extends StateMachineSubsystem<ClawState> {
  private final TalonFX motor;
  private final CANdi candi;
  private final Debouncer debouncer = RobotConfig.get().claw().debouncer();

  private boolean sensorRaw = false;
  private boolean sensorDebounced = false;

  public ClawSubsystem(TalonFX motor, CANdi candi) {
    super(SubsystemPriority.CLAW, ClawState.IDLE_NO_GP);

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
        motor.setVoltage(0.0);
      }
      case IDLE_W_CORAL -> {
        motor.setVoltage(0.0);
      }
      case INTAKING_ALGAE -> {
        motor.setVoltage(0.0);
      }
      case INTAKING_CORAL -> {
        motor.setVoltage(0.0);
      }
      case SCORE_ALGAE_NET -> {
        motor.setVoltage(0.0);
      }
      case SCORE_ALGAE_PROCESSOR -> {
        motor.setVoltage(0.0);
      }
      case SCORE_CORAL -> {
        motor.setVoltage(0.0);
      }
      case OUTTAKING -> {
        motor.setVoltage(0.0);
      }
    }
  }

  @Override
  public void whileInState(ClawState currentState) {
    DogLog.log("Claw/Motor/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Claw/Motor/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Claw/Motor/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Claw/Sensors/SensorRaw", sensorRaw);
    DogLog.log("Claw/Sensors/SensorDebounced", sensorDebounced);
  }
}
