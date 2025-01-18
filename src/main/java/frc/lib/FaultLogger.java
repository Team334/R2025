// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.FaultsTable.Fault;
import frc.lib.FaultsTable.FaultType;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

// (from team 1155 but slightly modified)

/**
 * FaultLogger allows for faults and errors to be logged and displayed.
 *
 * <pre>
 * FaultLogger.register(talonfx); // registers a TalonFX, periodically checking for hardware faults.
 * </pre>
 */
public final class FaultLogger {
  @FunctionalInterface
  public static interface FaultReporter {
    void report();
  }

  private static boolean _enableConsole = true;

  // DATA
  private static final List<FaultReporter> faultReporters = new ArrayList<>();
  private static final Set<Fault> newFaults = new HashSet<>();
  private static final Set<Fault> activeFaults = new HashSet<>();
  private static final Set<Fault> totalFaults = new HashSet<>();

  // NETWORK TABLES
  private static FaultsTable activeAlerts;
  private static FaultsTable totalAlerts;

  /** Must be called to setup the fault logger. */
  public static void setup(NetworkTableInstance ntInst) {
    var base = ntInst.getTable("Faults");

    activeAlerts = new FaultsTable(base, "Active Faults");
    totalAlerts = new FaultsTable(base, "Total Faults");
  }

  /** Must be called to setup the fault logger. */
  public static void setup() {
    setup(NetworkTableInstance.getDefault());
  }

  /** Whether or not to print new faults into the console. */
  public static void enableConsole(boolean enable) {
    _enableConsole = enable;
  }

  /** Polls registered fallibles. This method should be called periodically. */
  public static void update() {
    activeFaults.clear();

    faultReporters.forEach(f -> f.report());
    activeFaults.addAll(newFaults);
    newFaults.clear();

    // log to doglog as well
    activeFaults.forEach(f -> DogLog.logFault(f.toString()));

    totalFaults.addAll(activeFaults);

    // don't log to NT if there is a match going on (just use doglog)
    if (!DriverStation.isFMSAttached()) {
      activeAlerts.set(activeFaults);
      totalAlerts.set(totalFaults);
    }
  }

  /** Clears total faults. */
  public static void clear() {
    totalFaults.clear();
    activeFaults.clear();
    newFaults.clear();
  }

  /** Clears fault suppliers. */
  public static void unregisterAll() {
    faultReporters.clear();
  }

  /**
   * Returns the set of all current faults.
   *
   * @return The set of all current faults.
   */
  public static Set<Fault> activeFaults() {
    return activeFaults;
  }

  /**
   * Returns the set of all total faults.
   *
   * @return The set of all total faults.
   */
  public static Set<Fault> totalFaults() {
    return totalFaults;
  }

  /**
   * Reports a fault.
   *
   * @param fault The fault to report.
   */
  public static void report(Fault fault) {
    newFaults.add(fault);

    if (!_enableConsole) return;

    switch (fault.type()) {
      case ERROR -> DriverStation.reportError("[Fault Logger] " + fault.toString(), false);
      case WARNING -> DriverStation.reportWarning("[Fault Logger] " + fault.toString(), false);
      case INFO -> System.out.println("[Fault Logger] " + fault.toString());
    }
  }

  /**
   * Reports a fault.
   *
   * @param description The description of the fault.
   * @param type The type of the fault.
   */
  public static void report(String description, FaultType type) {
    report(new Fault(description, type));
  }

  /**
   * Registers a new fault supplier.
   *
   * @param supplier A supplier of an optional fault.
   */
  public static void register(Supplier<Optional<Fault>> supplier) {
    faultReporters.add(() -> supplier.get().ifPresent(FaultLogger::report));
  }

  /**
   * Registers a new fault supplier.
   *
   * @param condition Whether a failure is occuring.
   * @param description The failure's description.
   */
  public static void register(BooleanSupplier condition, String description, FaultType type) {
    faultReporters.add(
        () -> {
          if (condition.getAsBoolean()) {
            report(description, type);
          }
        });
  }

  /**
   * Registers a new TalonFX.
   *
   * @param talonFX The TalonFX.
   */
  public static void register(TalonFX talonFX) {
    String name = CTREUtil.getName(talonFX);

    register(
        () -> talonFX.getFault_Hardware().getValue(), name + "- Hardware Fault.", FaultType.ERROR);
    register(
        () -> talonFX.getFault_BootDuringEnable().getValue(),
        name + "- Boot While Enabling.",
        FaultType.WARNING);
    register(
        () -> talonFX.getFault_DeviceTemp().getValue(),
        name + "- Device Temperature Too High.",
        FaultType.WARNING);
    register(
        () -> talonFX.getFault_ProcTemp().getValue(),
        name + "- Processor Temp Too High.",
        FaultType.WARNING);
    register(
        () -> talonFX.getFault_Undervoltage().getValue(),
        name + "- Voltage Too Low, Check For Brownouts.",
        FaultType.WARNING);
  }

  /**
   * Registers a new CANcoder.
   *
   * @param cancoder The CANcoder.
   */
  public static void register(CANcoder cancoder) {
    String name = CTREUtil.getName(cancoder);

    register(
        () -> cancoder.getFault_Hardware().getValue(), name + "- Hardware Fault.", FaultType.ERROR);
    register(
        () -> cancoder.getFault_BadMagnet().getValue(),
        name + "- Bad Magnet Signal.",
        FaultType.ERROR);
    register(
        () -> cancoder.getFault_BootDuringEnable().getValue(),
        name + "- Boot While Enabling.",
        FaultType.WARNING);
    register(
        () -> cancoder.getFault_Undervoltage().getValue(),
        name + "- Voltage Too Low, Check For Brownouts.",
        FaultType.WARNING);
  }

  /**
   * Registers a new Pigeon.
   *
   * @param pigeon The Pigeon.
   */
  public static void register(Pigeon2 pigeon) {
    String name = CTREUtil.getName(pigeon);

    register(
        () -> pigeon.getFault_Hardware().getValue(), name + "- Hardware Fault.", FaultType.ERROR);
    register(
        () -> pigeon.getFault_BootDuringEnable().getValue(),
        name + "- Boot While Enabling.",
        FaultType.WARNING);
    register(
        () -> pigeon.getFault_Undervoltage().getValue(),
        name + "- Voltage Too Low, Check For Brownouts.",
        FaultType.WARNING);
  }

  /**
   * Registers a new PhotonCamera (more detailed logs on the web ui).
   *
   * @param photonCamera The PhotonCamera.
   */
  public static void register(PhotonCamera photonCamera) {
    register(
        () -> !photonCamera.isConnected(),
        photonCamera.getName() + ": Disconnected.",
        FaultType.ERROR);
  }
}
