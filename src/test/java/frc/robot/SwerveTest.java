package frc.robot;

import static frc.lib.UnitTestingUtil.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  public void close() throws Exception {
    reset(_swerve);
  }

  // @Test
  public void driveTo() {
    // TODO: ts don't work
    var goal = new Pose2d(0.5, 0, Rotation2d.fromDegrees(5));

    runToCompletion(_swerve.driveTo(goal));

    assertEquals(goal.getX(), _swerve.getPose().getX(), 0.2);
    assertEquals(goal.getY(), _swerve.getPose().getY(), 0.2);
    assertEquals(
        goal.getRotation().getRadians(),
        _swerve.getPose().getRotation().getRadians(),
        Math.toRadians(10));
  }
}
