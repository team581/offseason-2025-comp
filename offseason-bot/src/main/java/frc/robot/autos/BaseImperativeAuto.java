package frc.robot.autos;

import com.team581.autos.BaseAuto;
import com.team581.trailblazer.Trailblazer;
import com.team581.util.state_machines.StateMachine;
import frc.robot.robot_manager.RobotManager;

public abstract class BaseImperativeAuto<S extends Enum<S>> extends StateMachine<S>
    implements BaseAuto {
  protected final RobotManager robotManager;
  protected final Trailblazer trailblazer;

  protected BaseImperativeAuto(S initialState, RobotManager robotManager, Trailblazer trailblazer) {
    super(initialState);
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;
  }
}
