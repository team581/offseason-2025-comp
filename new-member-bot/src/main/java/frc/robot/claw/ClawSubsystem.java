package frc.robot.claw;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class ClawSubsystem extends StateMachineSubsystem<ClawState> {
  private final TalonFX motor;
  private double minVelocity;
  private double minVelocityTimeout;
  private final Timer timeout = new Timer();
  private final Debouncer debouncer;
  private boolean hasSeenMinVelocity = false;

  private boolean clawHasGp = false;

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
    return Robot.isSimulation() ? timeout(1.0) : clawHasGp;
  }

  private boolean getHasGp(double motorVelocity, double maxVelocity) {
    hasSeenMinVelocity =
        hasSeenMinVelocity
            || timeout.hasElapsed(minVelocityTimeout)
            || Math.abs(motorVelocity) >= minVelocity;

    return hasSeenMinVelocity && debouncer.calculate(Math.abs(motorVelocity) <= maxVelocity);
  }

  public void setState(ClawState newState) {
    setStateFromRequest(newState);
  }

  @Override
  protected void afterTransition(ClawState newState) {
    switch (newState) {
      case IDLE_NO_GP -> {
        reset();
        motor.disable();
      }
      case IDLE_W_ALGAE -> {
        motor.setVoltage(0.0);
      }
      case IDLE_W_CORAL -> {
        motor.setVoltage(0.0);
      }
      case INTAKING_ALGAE -> {
        reset();
        motor.setVoltage(0.0);
      }
      case INTAKING_CORAL -> {
        reset();
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
    clawHasGp =
        getHasGp(motor.getVelocity().getValueAsDouble(), RobotConfig.get().claw().gpMaxVelocity());
    DogLog.log("Claw/HasGP", clawHasGp);
    DogLog.log("Claw/Motor/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Claw/Motor/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Claw/Motor/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
  }
}
