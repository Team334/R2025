package frc.lib;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FaultsTable.Fault;
import frc.lib.FaultsTable.FaultType;
import java.util.HashSet;
import java.util.Set;

@Logged(strategy = Strategy.OPT_IN)
public abstract class AdvancedSubsystem extends SubsystemBase
    implements SelfChecked, AutoCloseable {
  // faults and the table containing them
  private final Set<Fault> _faults = new HashSet<Fault>();
  private final FaultsTable _faultsTable;

  private boolean _hasError = false;

  public AdvancedSubsystem() {
    this(NetworkTableInstance.getDefault());
  }

  public AdvancedSubsystem(NetworkTableInstance ntInst) {
    _faultsTable = new FaultsTable(ntInst.getTable("Self Check"), getName() + " Faults");
  }

  /**
   * Returns the name of the command that's currently requiring this subsystem. Is "None" when the
   * command in null.
   */
  public final String currentCommandName() {
    if (getCurrentCommand() != null) {
      return getCurrentCommand().getName();
    }

    return "None";
  }

  /** Adds a new fault under this subsystem. */
  protected final void addFault(String description, FaultType faultType) {
    _hasError = (faultType == FaultType.ERROR);

    Fault fault = new Fault(description, faultType);

    DogLog.logFault(fault.toString());

    _faults.add(fault);
    _faultsTable.set(_faults);
  }

  /** Clears this subsystem's faults. */
  public final void clearFaults() {
    _faults.clear();
    _faultsTable.set(_faults);

    _hasError = false;
  }

  /** Returns the faults belonging to this subsystem. */
  public final Set<Fault> getFaults() {
    return _faults;
  }

  /** Returns whether this subsystem contains the following fault. */
  public final boolean hasFault(String description, FaultType faultType) {
    return _faults.contains(new Fault(description, faultType));
  }

  /** Returns whether this subsystem has errors (has fault type of error). */
  public final boolean hasError() {
    return _hasError;
  }

  /** Returns a full Command that self checks this Subsystem for pre-match. */
  public final Command fullSelfCheck() {
    Command selfCheck =
        sequence(runOnce(this::clearFaults), selfCheck().until(this::hasError))
            .withName(getName() + " Self Check");

    return selfCheck;
  }

  @Override
  public void periodic() {
    DogLog.log(getName() + "/Current Command", currentCommandName());
    DogLog.log(getName() + "/Has Error", _hasError);
  }

  @Override
  public void close() {}
}
