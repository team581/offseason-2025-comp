package com.team581.simkit.internal;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

record SimMotor(TalonFX motor, ChassisReference orientation, double sensorToMechanismRatio) {
  static SimMotor of(TalonFX motor, ChassisReference orientation) {
    var config = new TalonFXConfiguration();
    motor.getConfigurator().refresh(config);
    var sensorToMechanismRatio = config.Feedback.SensorToMechanismRatio;

    return new SimMotor(motor, orientation, sensorToMechanismRatio);
  }

  /**
   * Applies a predicted mechanism state to the TalonFX simulation, optionally overriding the
   * orientation if one is provided.
   */
  void applyMechanismState(TrapezoidProfile.State state) {
    // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/simulation/simulation-intro.html#orientation
    var ratio = orientation == ChassisReference.Clockwise_Positive ? -1.0 : 1.0;

    motor.getSimState().setRawRotorPosition(state.position * sensorToMechanismRatio * ratio);
    motor.getSimState().setRotorVelocity(state.velocity * sensorToMechanismRatio * ratio);
  }
}
