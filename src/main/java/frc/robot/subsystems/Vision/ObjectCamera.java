package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;

/**
 * Simple camera for dashboard streaming
 */
public class ObjectCamera {
  private PhotonCamera m_camera;

  /**
   * Create Object Camera
   * @param name Name of device
   */
  public ObjectCamera(String name) {
    m_camera = new PhotonCamera(name);
  }

  /**
   * Get the PhotonCamera
   * @return The camera
   */
  public PhotonCamera getCamera() {
    return m_camera;
  }
}