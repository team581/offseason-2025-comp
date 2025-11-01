package frc.robot.claw;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class ClawSubsystem extends StateMachineSubsystem<ClawState> {
  private final TalonFX motor;
  private final Timer timeout = new Timer();
  private final Debouncer debouncer;
  private boolean hasSeenMinVelocity = false;
  private boolean velocityDetectsGp = false;

  private final DoubleSubscriber tunableMaxVelocity =
      DogLog.tunable("Claw/MaxVeloctiy", RobotConfig.get().claw().gpMaxVelocity());

  private final DoubleSubscriber tunableMinVelocity =
      DogLog.tunable("Claw/MinVelocity", RobotConfig.get().claw().gpMinVelocity());

  public ClawSubsystem(TalonFX motor) {
    super(SubsystemPriority.CLAW, ClawState.IDLE_NO_GP);

    motor.getConfigurator().apply(RobotConfig.get().claw().motorConfig());
    this.motor = motor;

    this.debouncer = RobotConfig.get().claw().debouncer();
    this.timeout.start();
  }

  @Override
  protected void collectInputs() {}

  public void reset() {
    hasSeenMinVelocity = false;
    timeout.reset();
  }

  public boolean getHasGP() {
    return Robot.isSimulation()
        ? switch (getState()) {
          case INTAKING_ALGAE, INTAKING_CORAL -> timeout(1.0);
          case SCORE_CORAL, SCORE_ALGAE_NET, SCORE_ALGAE_PROCESSOR, OUTTAKING_ALGAE, OUTTAKING_CORAL -> !timeout(1.0);
          case IDLE_NO_GP, UNTUNED -> false;
          case IDLE_W_ALGAE, IDLE_W_CORAL -> true;
        }
        : velocityDetectsGp;
  }

  private boolean getHasGp(double motorVelocity, boolean isCoral) {
    hasSeenMinVelocity =
        hasSeenMinVelocity
            || timeout.hasElapsed(RobotConfig.get().claw().minVelocityTimeout())
            || Math.abs(motorVelocity) >= tunableMinVelocity.get();

    return hasSeenMinVelocity
        && debouncer.calculate(Math.abs(motorVelocity) <= tunableMaxVelocity.get());
  }

  public void setState(ClawState newState) {
    setStateFromRequest(newState);
  }

  @Override
  protected void afterTransition(ClawState newState) {
    switch (newState) {
      case IDLE_NO_GP, INTAKING_CORAL, INTAKING_ALGAE -> reset();
      default -> {}
    }
    if (newState.volts == 0.0) {
      motor.disable();
    } else {
      motor.setVoltage(newState.volts);
    }
  }

  @Override
  public void whileInState(ClawState currentState) {
    velocityDetectsGp = getHasGp(motor.getVelocity().getValueAsDouble());
    DogLog.log("Claw/HasGP", getHasGP());
    DogLog.log("Claw/Motor/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble(), Volts);
    DogLog.log("Claw/Motor/StatorCurrent", motor.getStatorCurrent().getValueAsDouble(), Amps);
    DogLog.log("Claw/Motor/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble(), Amps);
  }
}
