package frc.robot.intake;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class IntakeSubsystem extends StateMachineSubsystem<IntakeState> {
  private final TalonFX motor;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  private double rawCurrent = 0.0;
  private double filteredCurrent = 0.0;

  private static final DoubleSubscriber JAM_CURRENT_THRESHOLD =
      DogLog.tunable("Intake/JamCurrentThreshold", 75.0);

  public IntakeSubsystem(TalonFX motor) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE);

    motor.getConfigurator().apply(RobotConfig.get().intake().motorConfig());
    this.motor = motor;
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case UNTUNED, STOPPED -> motor.disable();
      default -> motor.setVoltage(getState().volts);
    }
  }

  @Override
  protected void collectInputs() {
    rawCurrent = motor.getStatorCurrent().getValueAsDouble();
    filteredCurrent = currentFilter.calculate(rawCurrent);
  }

  public boolean isJammed() {
    return filteredCurrent > JAM_CURRENT_THRESHOLD.getAsDouble();
  }

  @Override
  public void whileInState(IntakeState currentState) {
    DogLog.log("Intake/Motor/FilteredCurrent", filteredCurrent, Amps);
  }

  public void setState(IntakeState newState) {
    setStateFromRequest(newState);
  }
}
