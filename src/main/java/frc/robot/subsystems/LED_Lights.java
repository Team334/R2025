// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FaultsTable.Fault;
import frc.lib.FaultsTable.FaultType;
import frc.robot.Constants;

import java.util.Map;
import java.util.Optional;

import dev.doglog.DogLog;

public class LED_Lights extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);

  private Color allianceColor;

  private LEDPattern _patternState;

  private enum DRIVERSTATE {
    DISABLED,
    TELEOP,
    PERIODIC,
    AUTON,
    TEST
  }

  private DRIVERSTATE lastState = DRIVERSTATE.DISABLED;

  /** Creates a new LED_Lights. */
  public LED_Lights() {
    m_led = new AddressableLED(9);
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      switch (ally.get()) {
        case Red:
          allianceColor = Color.kRed;
          break;
        case Blue:
          allianceColor = Color.kBlue;
          break;
        default:
          allianceColor = Constants.LEDConstants.errorColor;
          Fault fault = new Fault("Unable to get alliance color!", FaultType.ERROR);
          DogLog.logFault(fault.toString());
          break;
      }
    } else {
      allianceColor = Constants.LEDConstants.errorColor;
    }

    setDefaultCommand(runPattern(_patternState));
  }

  @Override
  public void simulationPeriodic() {
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    m_led.setData(m_ledBuffer);
  }

  public void stateLogic() {
    if (DriverStation.isTeleop() && lastState != DRIVERSTATE.TELEOP) {
      _patternState = LEDPattern.solid(allianceColor);
      lastState = DRIVERSTATE.TELEOP;
    } else if (DriverStation.isAutonomous() && lastState != DRIVERSTATE.AUTON) {
      LEDPattern base = LEDPattern.rainbow(255, 255);
      LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(25));
      _patternState = base.mask(mask);
      lastState = DRIVERSTATE.AUTON;
    } else if (DriverStation.isTest() && lastState != DRIVERSTATE.TEST) {
      LEDPattern base = LEDPattern.solid(Color.kRed).atBrightness(Percent.of(50));
      _patternState = base.blink(Seconds.of(1.5));
      lastState = DRIVERSTATE.TEST;
    } else {
      lastState = DRIVERSTATE.DISABLED;
      return;
    }
    _patternState.applyTo(m_ledBuffer);
  }

  public void aligning(String state) {
    LEDPattern pattern = LEDPattern.kOff;

    if (state.equals("Reef")) {
      pattern = LEDPattern.solid(Color.kOrange).blink(Seconds.of(0.15));
    } else if (state.equals("Human")) {
      pattern = LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.15));
    } else if (state.equals("Processor")) {
      pattern = LEDPattern.solid(Color.kMagenta).blink(Seconds.of(0.15));
    } else if(state.equals("Cage")){
      pattern = LEDPattern.solid(Color.kSkyBlue).blink(Seconds.of(0.15));
    }

    pattern.applyTo(m_ledBuffer);
  }

  public Command runPattern(LEDPattern pattern) {
    return run(() -> {
      stateLogic();
    });
  }

  public Command align(String state) {
    return run(() -> {
      aligning(state);
    });
  }
}
