package frc.robot;

import static frc.lib.UnitTestingUtil.*;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

public class SwerveTest {
  private Swerve _swerve;

  @BeforeEach
  public void setup() {
    setupTests();

    _swerve = TunerConstants.createDrivetrain();
  }

  @AfterEach
  public void close() {
    reset(_swerve);
  }
}
