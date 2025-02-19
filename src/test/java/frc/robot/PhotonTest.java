package frc.robot;

import static frc.lib.UnitTestingUtil.getNtInst;
import static frc.lib.UnitTestingUtil.reset;
import static frc.lib.UnitTestingUtil.setupTests;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;

// WHY IS THIS THROWING A PHOTON ERROR?
@Disabled
public class PhotonTest {
  private PhotonCamera _cam;
  private PhotonCameraSim _sim;

  @BeforeEach
  public void setup() {
    setupTests();

    _cam = new PhotonCamera(getNtInst(), "Cam");
    _sim = new PhotonCameraSim(_cam);
  }

  @AfterEach
  public void close() {
    reset(_cam, _sim);
  }

  @Test
  public void testOne() {
    System.out.println("Test One");
  }

  @Test
  public void testTwo() {
    System.out.println("Test Two");
  }
}
