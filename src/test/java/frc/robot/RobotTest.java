package frc.robot;

import static frc.lib.UnitTestingUtil.*;
import static org.junit.jupiter.api.Assertions.*;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.MultiBackend;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class RobotTest {
  private Robot _robot;

  @BeforeEach
  public void setup() {
    setupTests();

    _robot = new Robot(getNtInst());
  }

  @AfterEach
  public void close() throws Exception {
    DriverStationSim.setFmsAttached(false);
    DriverStationSim.notifyNewData();

    reset(_robot);
  }

  @Test
  public void fmsFileOnly() {
    // at the start should be both nt and file logging
    assert Epilogue.getConfig().backend instanceof MultiBackend; // multilogger setup
    assert DogLog.getOptions().ntPublish().getAsBoolean();

    DriverStationSim.setFmsAttached(true);
    DriverStationSim.notifyNewData();

    assert DriverStation.isFMSAttached();

    _robot.robotPeriodic();

    // once the fms connects, it should be file only
    assert Epilogue.getConfig().backend instanceof FileBackend;
    assertFalse(DogLog.getOptions().ntPublish());
  }
}
