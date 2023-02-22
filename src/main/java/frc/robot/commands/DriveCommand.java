// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */
  private final DriveSubsystem drive;
  
  private SlewRateLimiter translationXLimiter;
  private SlewRateLimiter translationYLimiter;
  private SlewRateLimiter rotationLimiter;

  private double translationXPercent;
  private double translationYPercent;
  private double rotationPercent;
  
  //private int gearShift = 1;

  public DriveCommand(DriveSubsystem drive) {
    this.drive = drive;

    translationXLimiter = new SlewRateLimiter(Constants.translationRateLimit);
    translationYLimiter = new SlewRateLimiter(Constants.translationRateLimit);
    rotationLimiter = new SlewRateLimiter(Constants.rotationRateLimit);

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    translationXPercent = -Constants.dXboxController.getRawAxis(XboxController.Axis.kLeftX.value);
    translationYPercent = -Constants.dXboxController.getRawAxis(XboxController.Axis.kLeftY.value);
    rotationPercent = Constants.dXboxController.getRawAxis(XboxController.Axis.kRightX.value);

    // Ignore minor stick movement
    if (Math.abs(translationXPercent) < Constants.deadzone) {
      translationXPercent = 0.0;
    }

    if (Math.abs(translationYPercent) < Constants.deadzone) {
      translationYPercent = 0.0;
    }

    if (Math.abs(rotationPercent) < Constants.deadzone) {
      rotationPercent = 0.0;
    }

    // Smooth out "jerky" transitions
    translationXPercent = translationXLimiter.calculate(translationXPercent);
    translationYPercent = translationYLimiter.calculate(translationYPercent);
    rotationPercent = rotationLimiter.calculate(rotationPercent);

    //gearShift = Constants.dXboxController.getRawButtonReleased(Constants.bXboxButton) && gearShift < 5 ? gearShift + 1 : Constants.dXboxController.getRawButtonReleased(Constants.xXboxButton) && gearShift > 1 ? gearShift - 1 : gearShift;

    // double multiplicationValue = gearShift * 0.2;
    // translationXPercent *= multiplicationValue;
    // translationYPercent *= multiplicationValue;
    // rotationPercent *= multiplicationValue;

      translationXPercent *= 1.00;
      translationYPercent *= 1.00;
      rotationPercent *= 1.00;
    // if(Constants.dXboxController.getRawButtonReleased(Constants.startXboxButton)){
    //   translationXPercent *= 11.00; //gotta crank it to 11
    //   translationYPercent *= 11.00;
    //   rotationPercent *= 11.00;
    // } 
    
      drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        translationXPercent * Constants.maxVelocity * drive.getTranslationPercent(),
        translationYPercent * Constants.maxVelocity * drive.getTranslationPercent(),
        rotationPercent * Constants.maxAngularVelocity * drive.getRotationPercent(),
        drive.getGyroscopeRotation()));
      /*
      drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        Math.pow(translationXPercent, Constants.controllerXYExpo) * translationXPercent * Constants.maxVelocity,
        Math.pow(translationYPercent, Constants.controllerXYExpo) * translationYPercent * Constants.maxVelocity,
        Math.pow(rotationPercent, Constants.controllerRoExpo) * translationYPercent * Constants.maxAngularVelocity,
        drive.getGyroscopeRotation()
        )
      );

       */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
