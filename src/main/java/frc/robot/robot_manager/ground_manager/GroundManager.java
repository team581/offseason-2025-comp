package frc.robot.robot_manager.ground_manager;

import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake_deploy.DeployState;
import frc.robot.intake_deploy.DeploySubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class GroundManager extends StateMachine<GroundState> {
    private final DeploySubsystem deploy;
    private final IntakeSubsystem intake;
    private boolean hasCoral = false;
    
    public GroundManager(DeploySubsystem deploy, IntakeSubsystem intake) {
        super(SubsystemPriority.GROUND_MANAGER, GroundState.IDLE_EMPTY);
        this.deploy = deploy;
        this.intake = intake;
    }

    @Override
    protected GroundState getNextState(GroundState currentState) {
        return switch (currentState) {
            case INTAKING -> hasCoral ? GroundState.IDLE_CORAL : currentState;
            case HANDOFF_RELEASE, L1_SCORE -> !hasCoral ? GroundState.IDLE_EMPTY : currentState;
            default -> currentState;
        };
    }

    @Override
    protected void afterTransition(GroundState newState) {
        switch (newState) {
            case IDLE_EMPTY -> {
                deploy.setState(DeployState.STOWED);
                intake.setState(IntakeState.IDLE_NO_GP);
            }
            case IDLE_CORAL -> {
                deploy.setState(DeployState.STOWED);
                intake.setState(IntakeState.IDLE_GP);
            }
            case INTAKING -> {
                deploy.setState(DeployState.FLOOR_INTAKE);
                intake.setState(IntakeState.INTAKING);
            }
            case L1_WAIT -> {
                deploy.setState(DeployState.L1_SCORE);
                intake.setState(IntakeState.IDLE_GP);
            }
            case L1_SCORE -> {
                deploy.setState(DeployState.L1_SCORE);
                intake.setState(IntakeState.SCORING);
            }
            case HANDOFF_WAIT -> {
                deploy.setState(DeployState.HANDOFF);
                intake.setState(IntakeState.IDLE_GP);
            }
            case HANDOFF_RELEASE -> {
                deploy.setState(DeployState.HANDOFF);
                intake.setState(IntakeState.CORAL_HANDOFF);
            }
            case UNJAM -> {
                deploy.setState(DeployState.UNJAM);
                intake.setState(IntakeState.UNJAM);
            }
        }
    }

    @Override
    protected void collectInputs() {
        hasCoral = intake.getHasGP();
    }

    public boolean hasCoral() {
        return hasCoral;
    }

    public void intakeRequest() {
        setStateFromRequest(GroundState.INTAKING);
    }

    public void l1Request() {
        switch (getState()) {
            default -> setStateFromRequest(GroundState.L1_WAIT);
            case L1_WAIT -> setStateFromRequest(GroundState.L1_SCORE);
        }
    }

    public void idleRequest() {
        if (hasCoral) {
            setStateFromRequest(GroundState.IDLE_CORAL);
        } else {
            setStateFromRequest(GroundState.IDLE_EMPTY);
        }
    }

    public void handoffWaitRequest() {
        setStateFromRequest(GroundState.HANDOFF_WAIT);
    }

    public void handoffReleaseRequest() {
        setStateFromRequest(GroundState.HANDOFF_RELEASE);
    }
}
