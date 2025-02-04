// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

public class LED_Lights extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);

  /** Creates a new LED_Lights. */
  public LED_Lights() {
    m_led = new AddressableLED(9);
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();

    setDefaultCommand(
        runPattern(
            LEDPattern.rainbow(255, 255)
                .mask(
                    LEDPattern.steps(maskSteps)
                        .scrollAtRelativeSpeed(Percent.per(Second).of(100)))));
  }

  @Override
  public void simulationPeriodic() {
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    m_led.setData(m_ledBuffer);
  }

  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_ledBuffer));
  }
}
