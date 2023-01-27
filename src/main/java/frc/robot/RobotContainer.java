// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController mXbox = new CommandXboxController(0);
  private final DriveSubsystem mDriveSubsystem = new DriveSubsystem(); 
  
	public RobotContainer() {
    SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(Constants.yRampSpeed);
    SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(Constants.xRampSpeed);
    SlewRateLimiter rSlewRateLimiter = new SlewRateLimiter(Constants.rRampSpeed);
    /* //funny stuff testing
     * () -> -modifyAxis(ySlewRateLimiter.calculate(Math.pow(mXbox.getLeftY() * Constants.controllerXYExpo))) * mDriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.maxSpeedY, 
       () -> -modifyAxis(xSlewRateLimiter.calculate(Math.pow(mXbox.getLeftx() * Constants.controllerXYExpo)) * mDriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.maxSpeedX, 
       () -> modifyAxis(ySlewRateLimiter.calculate(Math.pow(mXbox.getRightX() * Constants.controllerRoExpo))) * mDriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.maxSpeedRotation));
     */

    mDriveSubsystem.setDefaultCommand(new DriveCommand(
       mDriveSubsystem, 
       () -> -modifyAxis(mXbox.getLeftY()) * mDriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.maxSpeedY, 
       () -> -modifyAxis(mXbox.getLeftX()) * mDriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.maxSpeedX, 
       () -> modifyAxis(mXbox.getRightX()) * mDriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.maxSpeedRotation)); //the rotation axis is not inverted bc if it was trying to rotate right it would rotate left
  }
  private static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}

  private static double modifyAxis(double value) {
		// Deadband
		value = deadband(value, 0.1);
		// Square the axis
		value = Math.copySign(value * value, value);
  
		return value;
	}
/*
 * mDrivetrain.setDefaultCommand(new DrivetrainTeleOp(
				mDrivetrain,
				() -> -modifyAxis(mXbox.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
				() -> -modifyAxis(mXbox.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
				() -> -modifyAxis(mXbox.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
 */
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
  //}
  public void updateAngle() {
    mDriveSubsystem.updateAngle();
  }
}

