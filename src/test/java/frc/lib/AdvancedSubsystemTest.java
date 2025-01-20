// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.lib.UnitTestingUtil.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.FaultsTable.Fault;
import frc.lib.FaultsTable.FaultType;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class AdvancedSubsystemTest {
  private TestImpl _sub;

  @BeforeEach
  public void setup() {
    setupTests();

    _sub = new TestImpl();
  }

  @AfterEach
  public void close() {
    reset(_sub);
  }

  @Test
  public void addFault() {
    _sub.addFault("FAULT 1", FaultType.ERROR);

    assertEquals(1, _sub.getFaults().size());

    assert _sub.hasFault("FAULT 1", FaultType.ERROR);
  }

  @Test
  public void faultEquality() {
    Fault f1 = new Fault("FAULT 1", FaultType.ERROR);
    Fault f2 = new Fault("FAULT 1", FaultType.ERROR);

    assert f1.equals(f2);
  }

  @Test
  public void duplicateFaults() {
    _sub.addFault("FAULT 1", FaultType.ERROR);
    _sub.addFault("FAULT 1", FaultType.ERROR);

    assertEquals(1, _sub.getFaults().size());
  }

  @Test
  public void clearFaults() {
    _sub.addFault("FAULT 1", FaultType.WARNING);
    _sub.addFault("FAULT 2", FaultType.ERROR);

    _sub.clearFaults();

    assertEquals(0, _sub.getFaults().size());
  }

  @Test
  public void hasError() {
    _sub.addFault("FAULT 1", FaultType.ERROR);

    assert _sub.hasError();
  }

  @Test
  public void selfCheckFinish() {
    runToCompletion(_sub.fullSelfCheck());

    // should give FAULT 3 error
    assertEquals(3, _sub.getFaults().size());

    // should contain these faults
    assert _sub.hasFault("FAULT 1", FaultType.WARNING);
    assert _sub.hasFault("FAULT 2", FaultType.WARNING);
    assert _sub.hasFault("FAULT 3", FaultType.ERROR);
  }

  @Test
  public void currentCommandName() {
    assertEquals("None", _sub.currentCommandName()); // doing nothing

    var test = idle(_sub).withName("Test Command");

    run(test, 3);

    assertEquals("Test Command", _sub.currentCommandName());
  }

  public class TestImpl extends AdvancedSubsystem {
    public TestImpl() {
      super(UnitTestingUtil.getNtInst());
    }

    public double speed() {
      return 0;
    }

    @Override
    public Command selfCheck() {
      return shiftSequence(
          // check devices first
          runOnce(
              () -> {
                if (true) addFault("FAULT 1", FaultType.WARNING);
              }),
          runOnce(
              () -> {
                if (true) addFault("FAULT 2", FaultType.WARNING);
              }),
          runOnce(
              () -> {
                if (true) addFault("FAULT 3", FaultType.ERROR);
              }),

          // then check the subsystem
          runOnce(
              () -> {
                if (speed() < 2) addFault("TOO SLOW", FaultType.WARNING);
              }));
    }

    @Override
    public void close() {}
  }
}
