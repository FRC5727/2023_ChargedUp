// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.EnumMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  // Controls whether or not to update SmartDashboard
  private static final boolean armDebug = true;

  // Abstraction of the encode positions for a defined arm position
  private class ArmPosition {
    double lowerArmAngle;
    double upperArmAngle;

    public ArmPosition(double lower, double upper) {
      lowerArmAngle = lower;
      upperArmAngle = upper;
    }
  }

  // Enumeration of all defined arm positions
  public enum Position {
    NONE,
    CALIBRATION,
    STARTING,
    PRECHASSIS,
    CHASSIS,
    SAFE,
    GRID_LOW,
    GRID_MID,
    GRID_HIGH,
    INTAKE_PREGROUND,
    INTAKE_GROUND,
    INTAKE_SUBSTATION,
  };

  // Next positions for the Arm to target
  private Deque<Position> targetPosition = new ArrayDeque<Position>();
  private Position lastPosition = Position.STARTING;

  // Lookup "table" for each defined arm position
  private final EnumMap<Position, ArmPosition> armPositions = new EnumMap<>(Map.of(
      // All positions are lower arm first, upper arm second
      // Zero angles are with lower arm vertical and upper arm horizontal
      Position.STARTING, new ArmPosition(-20, -58),
      Position.PRECHASSIS, new ArmPosition(-32, -45),
      Position.CHASSIS, new ArmPosition(-20, -48),
      Position.SAFE, new ArmPosition(-19, 11),
      Position.GRID_LOW, new ArmPosition(-10, -43),
      Position.GRID_MID, new ArmPosition(-5, -4),
      Position.GRID_HIGH, new ArmPosition(28, 22),
      Position.INTAKE_PREGROUND, new ArmPosition(7, -56),
      Position.INTAKE_GROUND, new ArmPosition(11, -69),
      Position.INTAKE_SUBSTATION, new ArmPosition(-19, 11)
    ));

  private WPI_TalonFX lowerArmMaster;
  private WPI_TalonFX lowerArmSlave;

  private WPI_TalonFX upperArmMaster;
  private WPI_TalonFX upperArmSlave;

  private CANCoder lowerArmCoder;
  private CANCoder upperArmCoder;

  private PIDController lowerPidController;
  private PIDController upperPidController;

  // TODO Tune this
  private final PIDConstants lowerConstants = new PIDConstants(0.70, 0.00, 0.00);
  private final PIDConstants upperConstants = new PIDConstants(0.40, 0.00, 0.00);

  private double lowerMaxVoltage = 5;
  private double upperMaxVoltage = 4;

  public ArmSubsystem() {
    // Note that Map.of() only supports 10 key-value pairs, so calibration here
    armPositions.put(Position.CALIBRATION, new ArmPosition(-20, 0));

    this.lowerArmMaster = new WPI_TalonFX(Constants.Arm.lowerMaster, Constants.CANivoreName);
    this.lowerArmSlave = new WPI_TalonFX(Constants.Arm.lowerSlave, Constants.CANivoreName);
    this.lowerArmCoder = new CANCoder(Constants.Arm.lowCoder, Constants.CANivoreName);

    this.upperArmMaster = new WPI_TalonFX(Constants.Arm.upperMaster, Constants.CANivoreName);
    this.upperArmSlave = new WPI_TalonFX(Constants.Arm.upperSlave, Constants.CANivoreName);
    this.upperArmCoder = new CANCoder(Constants.Arm.upperCoder, Constants.CANivoreName);

    this.lowerPidController = new PIDController(lowerConstants.kP, lowerConstants.kI, lowerConstants.kD);
    this.upperPidController = new PIDController(upperConstants.kP, upperConstants.kI, upperConstants.kD);

    lowerPidController.disableContinuousInput();
    upperPidController.disableContinuousInput();

    lowerArmSlave.follow(lowerArmMaster);
    lowerArmSlave.setInverted(true);
    lowerArmMaster.setInverted(false);
    upperArmSlave.follow(upperArmMaster);
    upperArmSlave.setInverted(false);
    upperArmMaster.setInverted(true);

    brake();

    if (armDebug) {
      SmartDashboard.putData("Coast arm motors", Commands.startEnd(this::coast, this::brake, this));
    }
  }

  private void brake() {
    lowerArmMaster.setNeutralMode(NeutralMode.Brake);
    upperArmMaster.setNeutralMode(NeutralMode.Brake);
    lowerArmSlave.setNeutralMode(NeutralMode.Brake);
    upperArmSlave.setNeutralMode(NeutralMode.Brake);
    lowerArmMaster.stopMotor();
    upperArmMaster.stopMotor();
  }

  private void coast() {
    lowerArmMaster.setNeutralMode(NeutralMode.Coast);
    upperArmMaster.setNeutralMode(NeutralMode.Coast);
    lowerArmSlave.setNeutralMode(NeutralMode.Coast);
    upperArmSlave.setNeutralMode(NeutralMode.Coast);
  }

  // Limit a value (positive or negative)
  private double constrainValue(double value, double max) {
    return Math.signum(value) * Math.min(Math.abs(value), max);
  }

  private double normalizeAngle(double angle) {
    return (angle + 180) % 360 - 180;
  }

  public double getLowerAngle() {
    return normalizeAngle(lowerArmCoder.getAbsolutePosition() - Constants.Arm.lowerOffset + armPositions.get(Position.CALIBRATION).lowerArmAngle);
  }
  public double getUpperAngle() {
    return normalizeAngle(upperArmCoder.getAbsolutePosition() - Constants.Arm.highOffset + armPositions.get(Position.CALIBRATION).upperArmAngle);
  }

  public double getLowerAbsoluteAngle(){
    return lowerArmCoder.getAbsolutePosition();
  }

  public double getUpperAbsoluteAngle(){
    return upperArmCoder.getAbsolutePosition();
  }

  public void setTargetPosition(Position position) {
    targetPosition.clear();
    if (!Constants.Arm.positionDebugDirect && lastPosition != position) {
      // Determine first step based on last known position
      switch (lastPosition) {
        case STARTING:
          targetPosition.add(Position.PRECHASSIS);
        case PRECHASSIS:
        case CHASSIS:
          targetPosition.add(Position.CHASSIS);
          break;
        case GRID_LOW:
        case GRID_MID:
        case GRID_HIGH:
        case INTAKE_SUBSTATION:
        case SAFE:
          targetPosition.add(Position.SAFE);
          break;
        case INTAKE_PREGROUND:
        case INTAKE_GROUND:
          targetPosition.add(Position.INTAKE_PREGROUND);
          break;
        case CALIBRATION:
        case NONE:
          break;
      }

      // Now we know we are transitioning from either CHASSIS, SAFE, or INTAKE_PREGROUND
      // Determine any prequisites for final position
      switch (position) {
        case PRECHASSIS:
        case STARTING:
          if (targetPosition.isEmpty() || targetPosition.getLast() != Position.CHASSIS) {
            targetPosition.add(Position.CHASSIS);
            if (position == Position.STARTING) {
              targetPosition.add(Position.PRECHASSIS);
            }
          }
          break;
        case INTAKE_GROUND:
        case INTAKE_PREGROUND:
          if (targetPosition.isEmpty() || targetPosition.getLast() != Position.INTAKE_PREGROUND) {
            targetPosition.add(Position.GRID_LOW);
            targetPosition.add(Position.INTAKE_PREGROUND);
          }
          break;
        case GRID_MID:
        case GRID_HIGH:
        case INTAKE_SUBSTATION:
          if (targetPosition.isEmpty() || targetPosition.getLast() != Position.SAFE) {
            targetPosition.add(Position.SAFE);
          }
          break;
        case CHASSIS:
          if (targetPosition.isEmpty() || targetPosition.getLast() == Position.INTAKE_PREGROUND) {
            targetPosition.add(Position.GRID_LOW);
          }
          break;
        case GRID_LOW:
        case SAFE:
          // Safe directly from any of the previous positions
          break;
        case CALIBRATION:
        case NONE:
          break;  
      }
    }
    if (targetPosition.isEmpty() || targetPosition.getLast() != position) {
      targetPosition.add(position);
    }
  }

  public void beginMovement() {
    ArmPosition nextPosition = armPositions.get(targetPosition.peek());

    lowerPidController.setSetpoint(nextPosition.lowerArmAngle);
    upperPidController.setSetpoint(nextPosition.upperArmAngle);
    if (targetPosition.size() > 1) {
      if (targetPosition.peek() == Position.SAFE) {
        lowerPidController.setTolerance(25, 50);
        upperPidController.setTolerance(25, 50);
      } else {
        lowerPidController.setTolerance(3, 5);
        upperPidController.setTolerance(3, 5);
      }
    } else {
      lowerPidController.setTolerance(2, .5);
      upperPidController.setTolerance(2, .5);
    }
    lowerPidController.reset();
    upperPidController.reset();
  }

  public void updateMovement() {
    if (lowerPidController.atSetpoint() && upperPidController.atSetpoint()) {
      lastPosition = targetPosition.remove();
      if (!doneMovement()) {
        beginMovement();
      }
    }
    if (!doneMovement()) {
      lowerArmMaster.setVoltage(constrainValue(lowerPidController.calculate(getLowerAngle()), lowerMaxVoltage));
      upperArmMaster.setVoltage(constrainValue(upperPidController.calculate(getUpperAngle()), upperMaxVoltage));
    } else {
      stopMovement();
    }
  }

  public boolean doneMovement() {
    return targetPosition.isEmpty();
  }

  public void stopMovement() {
    lowerArmMaster.stopMotor();
    upperArmMaster.stopMotor();
  }

  public void lowerArmDirect(double voltage) {
    lowerArmMaster.setVoltage(voltage);
    if (voltage == 0) {
      lowerArmMaster.stopMotor();
    }
  }

  public void upperArmDirect(double voltage) {
    upperArmMaster.setVoltage(voltage);
    if (voltage == 0) {
      upperArmMaster.stopMotor();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update dashboard to help monitor and debug

    if (armDebug) {
      SmartDashboard.putNumber("Target queue depth", targetPosition.size());
      SmartDashboard.putString("Target Position", targetPosition.isEmpty() ? "<none>" : targetPosition.peek().toString());
      SmartDashboard.putNumber("Lower Arm Setpoint: ", lowerPidController.getSetpoint());
      SmartDashboard.putNumber("Upper Arm Setpoint: ", upperPidController.getSetpoint());
      SmartDashboard.putBoolean("Lower Arm set? ", lowerPidController.atSetpoint());
      SmartDashboard.putBoolean("Upper Arm set? ", upperPidController.atSetpoint());

      SmartDashboard.putNumber("Lower Angle ABSOLUTE", getLowerAbsoluteAngle());
      SmartDashboard.putNumber("Upper Angle ABSOLUTE", getUpperAbsoluteAngle());
      SmartDashboard.putNumber("Lower Angle", getLowerAngle());
      SmartDashboard.putNumber("Upper Angle", getUpperAngle());

      SmartDashboard.putNumber("Lower Arm Master Voltage:", lowerArmMaster.getMotorOutputVoltage());
      SmartDashboard.putNumber("Lower Arm Slave Voltage:", lowerArmSlave.getMotorOutputVoltage());

      SmartDashboard.putNumber("Upper Arm Master Voltage:", upperArmMaster.getMotorOutputVoltage());
      SmartDashboard.putNumber("Upper Arm Slave Voltage:", upperArmSlave.getMotorOutputVoltage());
    }
  }
}
