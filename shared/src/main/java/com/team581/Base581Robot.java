package com.team581;

import com.ctre.phoenix6.SignalLogger;
import com.team581.util.scheduling.SubsystemExecutionSequencer;
import com.team581.util.tuning.ElasticLayoutUtil;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public abstract class Base581Robot extends TimedRobot {
  private static final String FINALIZE_INIT_FAULT = "Robot finalizeInit() never called";

  private boolean isInitialized = false;

  public Base581Robot() {
    DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());

    SignalLogger.start();
    SignalLogger.setPath("/media/sda1/hoot/");

    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withNtPublish(GlobalConfig.IS_DEVELOPMENT)
            .withNtTunables(GlobalConfig.IS_DEVELOPMENT)
            .withUseLogThread(false));

    DogLog.log(
        "Metadata/RobotName",
        RobotKind.fromSerialNumber().map(RobotKind::toString).orElse("Unknown"));
    DogLog.log("Metadata/RoborioSerialNumber", RobotController.getSerialNumber());

    ElasticLayoutUtil.onBoot();
  }

  /** Must be called by subclasses to finalize initialization */
  protected void finalizeInit() {
    isInitialized = true;
    DogLog.clearFault(FINALIZE_INIT_FAULT);

    // This must be run before any commands are scheduled
    SubsystemExecutionSequencer.ready();

    configureBindings();
  }

  @Override
  public void robotInit() {
    if (!isInitialized) {
      DogLog.logFault(FINALIZE_INIT_FAULT);
    }
  }

  @Override
  public void disabledInit() {
    ElasticLayoutUtil.onDisable();
  }

  @Override
  public void autonomousInit() {
    ElasticLayoutUtil.onEnable();
  }

  @Override
  public void teleopInit() {
    ElasticLayoutUtil.onEnable();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void robotPeriodic() {
    DogLog.timeEnd("Scheduler/TimeSinceLastLoop");
    DogLog.time("Scheduler/TimeSinceLastLoop");

    DogLog.time("Scheduler/CommandSchedulerPeriodic");
    CommandScheduler.getInstance().run();
    DogLog.timeEnd("Scheduler/CommandSchedulerPeriodic");
    SubsystemExecutionSequencer.log();

    if (RobotController.getBatteryVoltage() < 12.5) {
      DogLog.logFault("Battery voltage low", AlertType.kWarning);
    } else {
      DogLog.clearFault("Battery voltage low");
    }
  }

  protected void logMetadata(
      String mavenName,
      String buildDate,
      String gitSha,
      String gitDate,
      String gitBranch,
      int gitDirty) {
    DogLog.log("Metadata/ProjectName", mavenName);
    DogLog.log("Metadata/BuildDate", buildDate);
    DogLog.log("Metadata/GitSHA", gitSha);
    DogLog.log("Metadata/GitDate", gitDate);
    DogLog.log("Metadata/GitBranch", gitBranch);

    switch (gitDirty) {
      case 0 -> DogLog.log("Metadata/GitDirty", "All changes committed");
      case 1 -> DogLog.log("Metadata/GitDirty", "Uncomitted changes");
      default -> DogLog.log("Metadata/GitDirty", "Unknown");
    }
  }

  protected abstract void configureBindings();
}
