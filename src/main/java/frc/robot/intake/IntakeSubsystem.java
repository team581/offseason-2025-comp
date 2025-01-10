package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class IntakeSubsystem extends StateMachine<IntakeState> {
    private final TalonFX motor;
    
    public IntakeSubsystem(TalonFX motor) {
        super(SubsystemPriority.INTAKE, IntakeState.IDLE);

        motor.getConfigurator().apply(RobotConfig.get().intake().motorConfig());

        this.motor = motor;
    }

    public void setState(IntakeState newState) {
        setStateFromRequest(newState);
    }

    @Override
    protected IntakeState getNextState(IntakeState currentState) {
        return currentState;
    }

    @Override
    protected void afterTransition(IntakeState newState) {
        switch (newState) {
            case IDLE -> {
                motor.setVoltage(0.0);
            }
            case INTAKING -> {
                motor.setVoltage(6.0);
            }
            case OUTTAKING -> {
                motor.setVoltage(-6.0);
            }
        }
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
    }
}
