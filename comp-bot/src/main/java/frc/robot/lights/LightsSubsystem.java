package frc.robot.lights;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.scheduling.SubsystemPriority;

public class LightsSubsystem extends StateMachineSubsystem<LightsState> {
  private final CANdle candle;
  private final SolidColor color = new SolidColor(0, 399).withUpdateFreqHz(50.0);
  private final StrobeAnimation blink = new StrobeAnimation(0, 399).withUpdateFreqHz(50.0);

  private LightsState storedState = LightsState.IDLE_EMPTY;
  private LightsState disabledState = LightsState.HOMED_SEES_TAGS;

  public LightsSubsystem(CANdle candle) {
    super(SubsystemPriority.LIGHTS, LightsState.IDLE_EMPTY);
    this.candle = candle;
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
  public void whileInState(LightsState currentState) {
    var usedState = DriverStation.isDisabled() ? disabledState : currentState;
    var RGBWColor = new RGBWColor(usedState.color);
    if (usedState.pattern == BlinkPattern.SOLID) {
      candle.setControl(color.withColor(RGBWColor));
    } else {
      candle.setControl(blink.withColor(RGBWColor).withFrameRate(1 / usedState.pattern.duration));
    }

    DogLog.log("Lights/Color", usedState.color.toString());
    DogLog.log("Lights/Pattern", usedState.pattern);
  }
}
