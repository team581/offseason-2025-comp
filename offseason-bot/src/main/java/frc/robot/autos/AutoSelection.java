package frc.robot.autos;

import com.team581.autos.AutoSelectionBase;
import com.team581.trailblazer.Trailblazer;
import frc.robot.autos.auto_state_machines.BlueCrossLineAuto;
import frc.robot.autos.auto_state_machines.RedCrossLineAuto;
import frc.robot.autos.auto_state_machines.StationAndLollipop5pcAutoDriveToPoint;
import frc.robot.autos.auto_state_machines.StationAndLollipop5pcAutoTrailblazer;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection implements AutoSelectionBase {
  CROSS_LINE(RedCrossLineAuto::new, BlueCrossLineAuto::new),
  STATION_LOLLIPOP_5PC_TRAILBLAZER(StationAndLollipop5pcAutoTrailblazer::new, StationAndLollipop5pcAutoTrailblazer::new),
  STATION_LOLLIPOP_5PC_DRIVE_TO_POSE(StationAndLollipop5pcAutoDriveToPoint::new, StationAndLollipop5pcAutoDriveToPoint::new);

  public final BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> redAuto;
  public final BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> blueAuto;

  private AutoSelection(
      BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> redAuto,
      BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> blueAuto) {
    this.redAuto = redAuto;
    this.blueAuto = blueAuto;
  }
}
