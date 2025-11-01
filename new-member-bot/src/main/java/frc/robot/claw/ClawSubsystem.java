package frc.robot.claw;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.mechanisms.VelocityDetector;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import frc.robot.Robot;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class ClawSubsystem extends StateMachineSubsystem<ClawState> {
  private final TalonFX motor;
  private double motorVelocity;
  private boolean coralDetectsGp = false;
  private boolean algaeDetectsGp = false;

  private final VelocityDetector coralDetector = RobotConfig.get().claw().coralDetector();;
  private final VelocityDetector algaeDetector = RobotConfig.get().claw().algaeDetector();;

  public ClawSubsystem(TalonFX motor) {
    super(SubsystemPriority.CLAW, ClawState.IDLE_NO_GP);

    motor.getConfigurator().apply(RobotConfig.get().claw().motorConfig());
    this.motor = motor;
  }

  @Override
  protected void collectInputs() {
    motorVelocity = motor.getVelocity().getValueAsDouble();
    coralDetectsGp = coralDetector.hasGamePiece(
        motorVelocity, RobotConfig.get().claw().coralMaxVelocity());
    algaeDetectsGp = algaeDetector.hasGamePiece(
        motorVelocity, RobotConfig.get().claw().algaeMaxVelocity());
  }

  public boolean getHasGP() {
    return Robot.isSimulation()
        ? switch (getState()) {
          case INTAKING_ALGAE, INTAKING_CORAL -> timeout(1.0);
          case SCORE_CORAL, SCORE_ALGAE_NET, SCORE_ALGAE_PROCESSOR, OUTTAKING_ALGAE, OUTTAKING_CORAL -> !timeout(1.0);
          case IDLE_NO_GP, UNTUNED -> false;
          case IDLE_W_ALGAE, IDLE_W_CORAL -> true;
        }
        : switch (getState()) {
          case INTAKING_ALGAE, IDLE_W_ALGAE, SCORE_ALGAE_NET, SCORE_ALGAE_PROCESSOR, OUTTAKING_ALGAE -> algaeDetectsGp;
          case INTAKING_CORAL, IDLE_W_CORAL, SCORE_CORAL, OUTTAKING_CORAL -> coralDetectsGp;
          case IDLE_NO_GP, UNTUNED -> false;
        };
  }

  public void setState(ClawState newState) {
    setStateFromRequest(newState);
  }

  @Override
  protected void afterTransition(ClawState newState) {
    switch (newState) {
      case IDLE_NO_GP, INTAKING_CORAL, INTAKING_ALGAE -> {
        coralDetector.reset();
        algaeDetector.reset();
      }
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
    DogLog.log("Claw/HasGP", getHasGP());
    DogLog.log("Claw/Motor/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble(), Volts);
    DogLog.log("Claw/Motor/StatorCurrent", motor.getStatorCurrent().getValueAsDouble(), Amps);
    DogLog.log("Claw/Motor/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble(), Amps);
  }
}
