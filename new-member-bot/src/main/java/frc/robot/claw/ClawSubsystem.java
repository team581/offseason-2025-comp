package frc.robot.claw;

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
  private double minVelocityTimeout;
  private final Timer timeout = new Timer();
  private final Debouncer debouncer;
  private boolean hasSeenMinVelocity = false;

  private boolean velocityDetectsGp = false;

  private DoubleSubscriber tunableMaxVelocity = DogLog.tunable("Claw/MaxVeloctiy", RobotConfig.get().claw().gpMaxVelocity());;
  private DoubleSubscriber tunableMinVelocity = DogLog.tunable("Claw/MinVelocity", 5.0);

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
          case SCORE_CORAL, SCORE_ALGAE_NET, SCORE_ALGAE_PROCESSOR, OUTTAKING -> !timeout(1.0);
          case IDLE_NO_GP, UNTUNED -> false;
          case IDLE_W_ALGAE, IDLE_W_CORAL -> true;
        }
        : velocityDetectsGp;
  }

  private boolean getHasGp(double motorVelocity, double maxVelocity) {
    hasSeenMinVelocity =
        hasSeenMinVelocity
            || timeout.hasElapsed(minVelocityTimeout)
            || Math.abs(motorVelocity) >= tunableMinVelocity.get();

    return hasSeenMinVelocity && debouncer.calculate(Math.abs(motorVelocity) <= maxVelocity);
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
    velocityDetectsGp = getHasGp(motor.getVelocity().getValueAsDouble(), tunableMaxVelocity.get());
    DogLog.log("Claw/HasGP", getHasGP());
    DogLog.log("Claw/Motor/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Claw/Motor/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Claw/Motor/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
  }
}
