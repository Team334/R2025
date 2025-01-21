package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;

public class Intake extends AdvancedSubsystem {
  private final Mechanism2d _mech = new Mechanism2d(1.85, 1);
  private final MechanismRoot2d _root = _mech.getRoot("pivot", 0.5, 0.1);

  private final MechanismLigament2d _intake =
      _root.append(new MechanismLigament2d("intake", 0.5, 0, 3, new Color8Bit(Color.kBlue)));

  private final TalonFX _feedMotor = new TalonFX(0); // TODO
  private final TalonFX _actuatorMotor = new TalonFX(0);

  public Intake() {
    setDefaultCommand(set(0.0, 0.0));
  }

  @Logged(name = "Speed")
  public double getSpeed() {
    return 0.0;
  }

  @Logged(name = "Angle")
  public double getAngle() {
    return 0.0;
  }

  /**
   * Set the actuator angle and feed speed.
   *
   * @param actuatorAngle Actuator angle in rad.
   * @param feedSpeed Feed wheel speed in rad/s.
   */
  public Command set(double actuatorAngle, double feedSpeed) {
    return run(() -> {}).withName("Set");
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    _intake.setAngle(Math.toDegrees(getAngle()));

    SmartDashboard.putData("Intake Visualizer", _mech);
  }

  @Override
  public void close() {
    _mech.close();
  }
}
