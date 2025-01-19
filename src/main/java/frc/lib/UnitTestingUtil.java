package frc.lib;

import static edu.wpi.first.units.Units.Seconds;

import dev.doglog.DogLog;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// (from team 1155)

/** Provides helper methods that run a command when performing unit tests. */
public class UnitTestingUtil {
  public static final Time TICK_RATE = Seconds.of(0.02);

  private static NetworkTableInstance _ntInst = null;

  /**
   * Returns the network tables instance for the current unit test. This is null when there is no
   * unit test running.
   */
  public static NetworkTableInstance getNtInst() {
    return _ntInst;
  }

  /** Sets up DS and initializes HAL with default values and asserts that it doesn't fail. */
  public static void setupTests() {
    assert HAL.initialize(500, 0);

    _ntInst = NetworkTableInstance.create();

    DriverStationSim.setEnabled(true);
    DriverStation.silenceJoystickConnectionWarning(true);
    DriverStationSim.notifyNewData();

    assert DriverStation.isEnabled();

    DogLog.setEnabled(false); // disabling doglog since it logs to the default nt instance

    FaultLogger.setup(_ntInst);

    FaultLogger.clear();
    FaultLogger.unregisterAll();
    FaultLogger.enableConsole(false);

    // delay 100 ms to wait for CTRE device enable
    Timer.delay(0.100);
  }

  /** Resets the CommandScheduler and the test NT instance. */
  public static void reset() {
    _ntInst.close();
    _ntInst = null;

    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * Resets CommandScheduler and NT and closes all closeables. Please call in an @AfterEach method!
   *
   * @param closeables All closeables that need to be closed.
   */
  public static void reset(AutoCloseable... closeables) {
    reset();

    for (AutoCloseable closeable : closeables) {
      try {
        closeable.close();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  /**
   * Runs CommandScheduler and updates timer repeatedly to fast forward subsystems and run commands.
   *
   * @param ticks The number of times CommandScheduler is run.
   */
  public static void fastForward(int ticks) {
    for (int i = 0; i < ticks; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(TICK_RATE.in(Seconds));
    }
  }

  /**
   * Fasts forward in time by running CommandScheduler and updating timer.
   *
   * @param time
   */
  public static void fastForward(Time time) {
    fastForward((int) (time.in(Seconds) / TICK_RATE.in(Seconds)));
  }

  /**
   * Runs CommandScheduler and updates timer to fast forward subsystems by 4 seconds and run
   * commands.
   */
  public static void fastForward() {
    fastForward(Seconds.of(4));
  }

  /**
   * Schedules and runs a command.
   *
   * @param command The command to run.
   */
  public static void run(Command command) {
    command.schedule();
    CommandScheduler.getInstance().run();
  }

  /**
   * Schedules and runs a command.
   *
   * @param command The command to run.
   * @param runs The number of times CommandScheduler is run.
   */
  public static void run(Command command, int runs) {
    command.schedule();
    fastForward(runs);
  }

  /**
   * Schedules a command and runs it until it ends. Be careful -- if the command you give never
   * ends, this will be an infinite loop!
   *
   * @param command
   */
  public static void runToCompletion(Command command) {
    command.schedule();

    fastForward(1);

    while (command.isScheduled()) {
      fastForward(1);
    }
  }
}
