package frc.omegabytes.library.OmegaLib.ControllerTypeBeat;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * A {@link Button} that gets its state from a {@link GenericHID}.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class JoystickButton extends Button {
  /**
   * Creates a joystick button for triggering commands.
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */
  public JoystickButton(GenericHID joystick, int buttonNumber) {
    super(() -> joystick.getRawButton(buttonNumber));
    requireNonNullParam(joystick, "joystick", "JoystickButton");
  }
}
