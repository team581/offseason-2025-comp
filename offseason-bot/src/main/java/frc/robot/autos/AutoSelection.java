package frc.robot.autos;

import com.team581.autos.AutoSelectionBase;
import com.team581.trailblazer.Trailblazer;
import frc.robot.autos.auto_state_machines.BlueCrossLineAuto;
import frc.robot.autos.auto_state_machines.DoNothingAuto;
import frc.robot.autos.auto_state_machines.NonProcessorSideStationAuto4pc;
import frc.robot.autos.auto_state_machines.NonProcessorSideStationAuto5pc;
import frc.robot.autos.auto_state_machines.NonProcessorSideStationLollipop5pc;
import frc.robot.autos.auto_state_machines.ProcessorSideStationAuto4pc;
import frc.robot.autos.auto_state_machines.ProcessorSideStationAuto5pc;
import frc.robot.autos.auto_state_machines.ProcessorSideStationLollipop5pc;
import frc.robot.autos.auto_state_machines.RedCrossLineAuto;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection implements AutoSelectionBase {
  DO_NOTHING(DoNothingAuto::new, DoNothingAuto::new),
  CROSS_LINE(RedCrossLineAuto::new, BlueCrossLineAuto::new),
  NON_PROCESSOR_SIDE_STATION_4PC(
      NonProcessorSideStationAuto4pc::new, NonProcessorSideStationAuto4pc::new),
  NON_PROCESSOR_SIDE_STATION_5PC(
      NonProcessorSideStationAuto5pc::new, NonProcessorSideStationAuto5pc::new),
  NON_PROCESSOR_SIDE_STATION_LOLLIPOP_5PC(
      NonProcessorSideStationLollipop5pc::new, NonProcessorSideStationLollipop5pc::new),

  PROCESSOR_SIDE_STATION_4PC(ProcessorSideStationAuto4pc::new, ProcessorSideStationAuto4pc::new),
  PROCESSOR_SIDE_STATION_5PC(ProcessorSideStationAuto5pc::new, ProcessorSideStationAuto5pc::new),
  PROCESSOR_SIDE_STATION_LOLLIPOP_5PC(
      ProcessorSideStationLollipop5pc::new, ProcessorSideStationLollipop5pc::new);

  public final BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> redAuto;
  public final BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> blueAuto;

  private AutoSelection(
      BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> redAuto,
      BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> blueAuto) {
    this.redAuto = redAuto;
    this.blueAuto = blueAuto;
  }
}
