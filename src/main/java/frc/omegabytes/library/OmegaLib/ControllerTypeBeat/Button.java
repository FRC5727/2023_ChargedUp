package frc.omegabytes.library.OmegaLib.ControllerTypeBeat;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.BooleanSupplier;
public class Button extends Trigger {
    /**
     * Default constructor; creates a button that is never pressed.
     *
     *
     */

    public Button() {}
  
    /**
     * Creates a new button with the given condition determining whether it is pressed.
     *
     * @param isPressed returns whether the trigger should be active
     */
    
    public Button(BooleanSupplier isPressed) {
      super(isPressed);
    }
  
    /**
     * Starts the given command whenever the button is newly pressed.
     *
     * @param command the command to start
     * @return this button, so calls can be chained
     * 
     */
    
    public Button whenPressed(final Command command) {
      whenActive(command);
      return this;
    }
  
    /**
     * Runs the given runnable whenever the button is newly pressed.
     *
     * @param toRun the runnable to run
     * @param requirements the required subsystems
     * @return this button, so calls can be chained
     *
     */
    
    public Button whenPressed(final Runnable toRun, Subsystem... requirements) {
      whenActive(toRun, requirements);
      return this;
    }
  
    /**
     * Constantly starts the given command while the button is held.
     *
     * <p>{@link Command#schedule()} will be called repeatedly while the button is held, and will be
     * canceled when the button is released.
     *
     * @param command the command to start
     * @return this button, so calls can be chained
     
     */
    public Button whileHeld(final Command command) {
      whileActiveContinuous(command);
      return this;
    }
  
    /**
     * Constantly runs the given runnable while the button is held.
     *
     * @param toRun the runnable to run
     * @param requirements the required subsystems
     * @return this button, so calls can be chained
     */
    public Button whileHeld(final Runnable toRun, Subsystem... requirements) {
      whileActiveContinuous(toRun, requirements);
      return this;
    }
  
    /**
     * Starts the given command when the button is first pressed, and cancels it when it is released,
     * but does not start it again if it ends or is otherwise interrupted.
     *
     * @param command the command to start
     * @return this button, so calls can be chained
     */
    public Button whenHeld(final Command command) {
      whileActiveOnce(command);
      return this;
    }
  
    /**
     * Starts the command when the button is released. The command is set to be interruptible.
     *
     * @param command the command to start
     * @return this button, so calls can be chained
     */
    public Button whenReleased(final Command command) {
      whenInactive(command);
      return this;
    }
  
    /**
     * Runs the given runnable when the button is released.
     *
     * @param toRun the runnable to run
     * @param requirements the required subsystems
     * @return this button, so calls can be chained
     */
    public Button whenReleased(final Runnable toRun, Subsystem... requirements) {
      whenInactive(toRun, requirements);
      return this;
    }
  
    /**
     * Toggles the command whenever the button is pressed (on, then off, then on). The command is set
     * to be interruptible.
     *
     * @param command the command to start
     * @return this button, so calls can be chained
     */
    public Button toggleWhenPressed(final Command command) {
      toggleWhenActive(command);
      return this;
    }
  
    /**
     * Cancels the command when the button is pressed.
     *
     * @param command the command to start
     * @return this button, so calls can be chained
     */
    public Button cancelWhenPressed(final Command command) {
      cancelWhenActive(command);
      return this;
    }
  }
