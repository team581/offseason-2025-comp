package frc.robot.autos;

import com.team581.autos.AutoSelectionBase;
import com.team581.trailblazer.Trailblazer;
import frc.robot.autos.auto_state_machines.BlueCrossLineAuto;
import frc.robot.autos.auto_state_machines.DoNothingAuto;
import frc.robot.autos.auto_state_machines.RedCrossLineAuto;
import frc.robot.autos.auto_state_machines.StationAndLollipop5pcAuto;
import frc.robot.autos.auto_state_machines.StationAuto4pc;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection implements AutoSelectionBase {
  DO_NOTHING(DoNothingAuto::new, DoNothingAuto::new),
  CROSS_LINE(RedCrossLineAuto::new, BlueCrossLineAuto::new),
  STATION_LOLLIPOP_5PC(StationAndLollipop5pcAuto::new, StationAndLollipop5pcAuto::new),
  STATION_AUTO_4PC(StationAuto4pc::new, StationAuto4pc::new);

  public final BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> redAuto;
  public final BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> blueAuto;

  private AutoSelection(
      BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> redAuto,
      BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> blueAuto) {
    this.redAuto = redAuto;
    this.blueAuto = blueAuto;
  }
}
