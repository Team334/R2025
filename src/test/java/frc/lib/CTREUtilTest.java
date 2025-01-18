package frc.lib;

import static frc.lib.UnitTestingUtil.*;
import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.FaultsTable.Fault;
import frc.lib.FaultsTable.FaultType;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class CTREUtilTest {
  private TalonFX _motor;
  private Pigeon2 _gyro;
  private CANcoder _encoder;

  @BeforeEach
  public void setup() {
    setupTests();

    _motor = new TalonFX(1);
    _gyro = new Pigeon2(2);
    _encoder = new CANcoder(3);
  }

  @AfterEach
  public void close() throws Exception {
    reset();

    _motor.close();
    _encoder.close();
    _gyro.close();
  }

  @Test
  public void motorName() {
    var name = CTREUtil.getName(_motor);

    assertEquals("TalonFX (1)", name);
  }

  @Test
  public void gyroName() {
    var name = CTREUtil.getName(_gyro);

    assertEquals("Pigeon (2)", name);
  }

  @Test
  public void encoderName() {
    var name = CTREUtil.getName(_encoder);

    assertEquals("CANcoder (3)", name);
  }

  @Test
  public void attempt() {
    // if all names work then, attempt will work for any device
    var name = "TalonFX (3)";

    var failed1 = CTREUtil.attempt(() -> StatusCode.ConfigFailed, name);
    var failed2 = CTREUtil.attempt(() -> StatusCode.OK, name);

    assert failed1;
    assert !failed2;

    FaultLogger.update();

    assert FaultLogger.totalFaults()
        .contains(
            new Fault(
                name + " - Config Apply Failed - " + StatusCode.ConfigFailed.getDescription(),
                FaultType.ERROR));

    assert FaultLogger.totalFaults()
        .contains(new Fault(name + " - Config Apply Successful.", FaultType.INFO));
  }
}
