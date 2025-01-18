package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static frc.lib.UnitTestingUtil.*;
import static org.junit.jupiter.api.Assertions.*;

import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class WheelRadiusCharacterizationTest {
  // chassis omega during characterization (TODO: this would be tuneable later)
  private final double velOmega = 1;

  private Swerve _swerve;
  private WheelRadiusCharacterization _characterization;

  @BeforeEach
  public void setup() {
    setupTests();

    _swerve = TunerConstants.createDrivetrain();
    _characterization = new WheelRadiusCharacterization(_swerve);
  }

  @AfterEach
  public void close() throws Exception {
    reset(_swerve);
  }

  @Test
  public void wheelRadiusCharacterization() {
    // run enough ticks to complete 4pi radians at the given characterization omega
    run(_characterization, (int) ((Math.PI * 4 / velOmega) / TICK_RATE.in(Seconds)));

    // accept with a tolerance of 0.002 meters (i think that's good?)
    assertEquals(
        TunerConstants.FrontLeft.WheelRadius, _characterization.getWheelRadius().in(Meters), 3e-3);
  }
}
