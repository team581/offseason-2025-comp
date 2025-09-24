package frc.robot.lights;

import com.ctre.phoenix6.hardware.CANdle;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.scheduling.SubsystemPriority;

public class LightsSubsystem extends StateMachine<LightsState> {
  private final CANdle candle;

  private final Timer blinkTimer = new Timer();
  private LightsState storedState = LightsState.IDLE_EMPTY;
  private LightsState disabledState = LightsState.HOMED_SEES_TAGS;

  public LightsSubsystem(CANdle candle) {
    super(SubsystemPriority.LIGHTS, LightsState.IDLE_EMPTY);
    this.candle = candle;
    blinkTimer.start();
  }

  public void setState(LightsState newState) {
    setStateFromRequest(newState);
  }

  public void blink() {
    storedState = getState();
    setStateFromRequest(LightsState.BLINK);
  }

  public void setDisabledState(LightsState newDisabledState) {
    disabledState = newDisabledState;
  }

  @Override
  protected LightsState getNextState(LightsState currentState) {
    return switch (currentState) {
      case BLINK -> timeout(1.0) ? storedState : currentState;
      default -> currentState;
    };
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
  }
  @Override
  protected void afterTransition(LightsState newState) {
      // TODO Auto-generated method stub
  }
}
