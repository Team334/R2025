// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringArrayPublisher;
import java.util.Set;

// (from 1155)

/** A table on Network Tables that contains non-persistent faults. */
public class FaultsTable {
  private final StringArrayPublisher errors;
  private final StringArrayPublisher warnings;
  private final StringArrayPublisher infos;

  /** An individual fault, containing necessary information. */
  public static record Fault(String description, FaultType type) {
    @Override
    public final boolean equals(Object other) {
      if (other == this) return true;

      if (other instanceof Fault o) {
        return o.description.equals(description) && o.type.equals(type);
      }

      return false;
    }

    @Override
    public final String toString() {
      return "Fault - " + description + ", Type - " + type.toString();
    }
  }

  /**
   * The type of fault, used for detecting whether the fallible is in a failure state and displaying
   * to NetworkTables.
   */
  public static enum FaultType {
    INFO,
    WARNING,
    ERROR
  }

  public FaultsTable(NetworkTable base, String name) {
    NetworkTable table = base.getSubTable(name);

    table.getStringTopic(".type").publish().set("Alerts"); // set to alerts widget

    errors = table.getStringArrayTopic("errors").publish();
    warnings = table.getStringArrayTopic("warnings").publish();
    infos = table.getStringArrayTopic("infos").publish();
  }

  public void set(Set<Fault> faults) {
    errors.set(filteredStrings(faults, FaultType.ERROR));
    warnings.set(filteredStrings(faults, FaultType.WARNING));
    infos.set(filteredStrings(faults, FaultType.INFO));
  }

  // Returns an array of descriptions of all faults that match the specified type.
  private String[] filteredStrings(Set<Fault> faults, FaultType type) {
    return faults.stream()
        .filter(a -> a.type() == type)
        .map(Fault::description)
        .toArray(String[]::new);
  }
}
