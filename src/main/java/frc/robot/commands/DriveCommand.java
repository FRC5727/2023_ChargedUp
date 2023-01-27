// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */
  /** Creates a new DriveCommand. */
  public final DriveSubsystem mDrivesSubsystem;
  private final DoubleSupplier mTranslationXSupplier;
  private final DoubleSupplier mTranslationYSupplier;
  private final DoubleSupplier mRotationSupplier;
  public DriveCommand(DriveSubsystem mDriveSubsystem, DoubleSupplier translationXSupplier,
  DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
    mDrivesSubsystem = mDriveSubsystem;
    mTranslationXSupplier = translationXSupplier;
    mTranslationYSupplier = translationYSupplier;
    mRotationSupplier = rotationSupplier;

    addRequirements(mDriveSubsystem);
  }


  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      mDrivesSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        mTranslationXSupplier.getAsDouble(),
        mTranslationYSupplier.getAsDouble(),
        mRotationSupplier.getAsDouble(),
        mDrivesSubsystem.getGyroscopeRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
