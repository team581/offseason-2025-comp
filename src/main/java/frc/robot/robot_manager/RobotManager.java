package frc.robot.robot_manager;

import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.VisionSubsystem;

public class RobotManager extends StateMachine<RobotState> {
    public final VisionSubsystem vision;
    public final ImuSubsystem imu;
    public final LocalizationSubsystem localization;
    public final SwerveSubsystem swerve;

    public RobotManager(
        VisionSubsystem vision, 
        ImuSubsystem imu, 
        LocalizationSubsystem localization, 
        SwerveSubsystem swerve) {
        super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE_NO_GP);
        this.vision = vision;
        this.imu = imu;
        this.localization = localization;
        this.swerve = swerve;
    }

    @Override
    protected void collectInputs() {

    }
    
    @Override
    protected RobotState getNextState(RobotState currentState) {
        return switch (currentState) {
            default -> currentState;
        };
    }

    @Override
    protected void afterTransition(RobotState newState) {
        switch (newState) {
            default -> {}
        }
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
    }

    public void confirmScore() {
        switch (getState()) {
            case 
            CLIMBING_1_LINEUP, 
            CLIMBING_2_HANGING, 
            INTAKE_ALGAE_FLOOR, 
            INTAKE_ALGAE_L2, 
            INTAKE_ALGAE_L3,
            DISLODGE_ALGAE_L2,
            DISLODGE_ALGAE_L3,
            INTAKE_CORAL_FLOOR_HORIZONTAL,
            INTAKE_CORAL_FLOOR_UPRIGHT,
            INTAKE_CORAL_STATION -> {}

            case PROCESSOR_WAITING, IDLE_ALGAE -> setStateFromRequest(RobotState.PROCESSOR_PREPARE_TO_SCORE);
            case NET_WAITING -> setStateFromRequest(RobotState.NET_PREPARE_TO_SCORE);

            case CORAL_L1_WAITING -> setStateFromRequest(RobotState.CORAL_L1_PREPARE_TO_SCORE);
            case CORAL_L2_WAITING -> setStateFromRequest(RobotState.CORAL_L2_PREPARE_TO_SCORE);
            case CORAL_L3_WAITING -> setStateFromRequest(RobotState.CORAL_L3_PREPARE_TO_SCORE);
            case CORAL_L4_WAITING -> setStateFromRequest(RobotState.CORAL_L4_PREPARE_TO_SCORE);
            
            //change default coral score level or algea score if needed
            default -> setStateFromRequest(RobotState.CORAL_L2_PREPARE_TO_SCORE);
        }
    }
}
