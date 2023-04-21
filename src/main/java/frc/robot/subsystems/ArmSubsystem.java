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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Dashboard;
import frc.robot.subsystems.LED.Colors;

public class ArmSubsystem extends SubsystemBase {
  // Controls whether or not to update SmartDashboard
  private boolean armDebug = false;

  // Controls whether or not to move arm to manual positions
  private boolean armDirectDebug = false;

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
    CHASSIS,
    CHASSIS_CONE,
    SAFE,
    GRID_LOW,
    GRID_MID,
    GRID_HIGH,
    INTAKE_PREGROUND,
    INTAKE_GROUND,
    INTAKE_SUBSTATION,
    YOSHI,
  };

  // Next positions for the Arm to target
  private Deque<Position> targetPosition = new ArrayDeque<Position>();
  private Position lastPosition = Position.STARTING;

  // Lookup "table" for each defined arm position
  private final EnumMap<Position, ArmPosition> armPositions = new EnumMap<>(Map.ofEntries(
      // All positions are lower arm first, upper arm second
      // Zero angles are with lower arm vertical and upper arm horizontal
      Map.entry(Position.CALIBRATION, new ArmPosition(-20, 0)),
      Map.entry(Position.STARTING, new ArmPosition(-21, -60)),
      Map.entry(Position.CHASSIS, new ArmPosition(-27, -44)),
      Map.entry(Position.CHASSIS_CONE, new ArmPosition(-27, -52)),
      Map.entry(Position.SAFE, new ArmPosition(-27, 22)),
      Map.entry(Position.GRID_LOW, new ArmPosition(-23, -33)),
      Map.entry(Position.GRID_MID, new ArmPosition(-7, -7)),
      Map.entry(Position.GRID_HIGH, new ArmPosition(26, 22)),
      Map.entry(Position.INTAKE_PREGROUND, new ArmPosition(0, -44)),
      Map.entry(Position.INTAKE_GROUND, new ArmPosition(12, -70)),
      Map.entry(Position.INTAKE_SUBSTATION, new ArmPosition(-27, 16)),
      Map.entry(Position.YOSHI, new ArmPosition(50, -36))
    ));

  private LED s_Led;
  private IntakeSubsystem s_Intake;

  private WPI_TalonFX lowerArmMaster;
  private WPI_TalonFX lowerArmSlave;

  private WPI_TalonFX upperArmMaster;
  private WPI_TalonFX upperArmSlave;

  private CANCoder lowerArmCoder;
  private CANCoder upperArmCoder;

  private ProfiledPIDController lowerPidController;
  private ProfiledPIDController upperPidController;

  private final PIDConstants lowerConstants = new PIDConstants(0.58, 0.00, 0.00);
  private final PIDConstants lowerConstantsLaden = new PIDConstants(0.58, 0.00, 0.00);
  private final PIDConstants upperConstantsTeleOp = new PIDConstants(0.58, 0.00, 0.00);
  private final PIDConstants upperConstantsAuto = new PIDConstants(0.47, 0.00, 0.00);
  private final PIDConstants upperConstantsLaden = new PIDConstants(0.47, 0.00, 0.00);
  private final TrapezoidProfile.Constraints lowerConstraints = new TrapezoidProfile.Constraints(180, 270);
  private final TrapezoidProfile.Constraints upperConstraints = new TrapezoidProfile.Constraints(180, 400);

  private double lowerMaxVoltage = 12;
  private double upperMaxVoltage = 12;

  public ArmSubsystem(LED s_Led, IntakeSubsystem s_Intake) {
    this.s_Led = s_Led;
    this.s_Intake = s_Intake;

    this.lowerArmMaster = new WPI_TalonFX(Constants.Arm.lowerMaster, "rio");
    this.lowerArmSlave = new WPI_TalonFX(Constants.Arm.lowerSlave, "rio");
    this.lowerArmCoder = new CANCoder(Constants.Arm.lowCoder, "rio");

    this.upperArmMaster = new WPI_TalonFX(Constants.Arm.upperMaster, "rio");
    this.upperArmSlave = new WPI_TalonFX(Constants.Arm.upperSlave, "rio");
    this.upperArmCoder = new CANCoder(Constants.Arm.upperCoder, "rio");

    this.lowerPidController = new ProfiledPIDController(lowerConstants.kP, lowerConstants.kI, lowerConstants.kD, lowerConstraints);
    this.upperPidController = new ProfiledPIDController(upperConstantsAuto.kP, upperConstantsAuto.kI, upperConstantsAuto.kD, upperConstraints);

    lowerPidController.disableContinuousInput();
    upperPidController.disableContinuousInput();

    lowerArmSlave.follow(lowerArmMaster);
    lowerArmSlave.setInverted(true);
    lowerArmMaster.setInverted(false);
    upperArmSlave.follow(upperArmMaster);
    upperArmSlave.setInverted(false);
    upperArmMaster.setInverted(true);

    SmartDashboard.putData("Coast arm motors",
      Commands.startEnd(() -> { s_Led.setRainbow(); coast(); },
                        () -> { brake(); s_Led.setColor(Colors.omegabytes); },
                        this));

    Dashboard.watchBoolean("Arm debug", armDebug, (val) -> armDebug = val.booleanValue());
    Dashboard.watchBoolean("Arm direct debug", armDirectDebug, (val) -> armDirectDebug = val.booleanValue());

    brake();
  }

  public boolean isDirectMode() {
    return armDirectDebug;
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
    Position lastTarget = targetPosition.isEmpty() ? null : targetPosition.getFirst();
    
    targetPosition.clear();
    
    if (!armDirectDebug && lastPosition != position && lastTarget != position) {
      // Determine first step based on last known position
      switch (lastPosition) {
        case GRID_HIGH:
          if (position != Position.YOSHI && !DriverStation.isAutonomous()) {
            targetPosition.add(Position.SAFE);
          }
          break;
        case SAFE:
          if (lastTarget == Position.GRID_HIGH && position != Position.YOSHI && !DriverStation.isAutonomous()) {
            targetPosition.add(Position.SAFE);
          }
          break;
        case INTAKE_GROUND:
          targetPosition.add(Position.INTAKE_PREGROUND);
          break;
        case INTAKE_PREGROUND:
          if (lastTarget == Position.INTAKE_GROUND) {
            targetPosition.add(Position.INTAKE_PREGROUND);
          }
          break;
        case STARTING:
        case CHASSIS:
        case CHASSIS_CONE:
        case GRID_MID:
        case GRID_LOW:
        case INTAKE_SUBSTATION:
        case CALIBRATION:
        case YOSHI:
        case NONE:
          break;
      }

      // Now we know we are transitioning from a safe state
      // Determine any prequisites for final position
      switch (position) {
        case INTAKE_GROUND:
          if (targetPosition.isEmpty() || targetPosition.getLast() != Position.INTAKE_PREGROUND) {
            targetPosition.add(Position.INTAKE_PREGROUND);
          }
          break;
        case GRID_HIGH:
          if (targetPosition.isEmpty() || targetPosition.getLast() != Position.SAFE) {
            if (lastPosition != Position.YOSHI) {
              targetPosition.add(Position.SAFE);
            }
          }
          break;
        case STARTING:
        case INTAKE_PREGROUND:
        case INTAKE_SUBSTATION:
        case CHASSIS:
        case CHASSIS_CONE:
        case GRID_MID:
        case GRID_LOW:
        case SAFE:
          // Safe directly from any of the previous positions
          break;
        case YOSHI:
        case CALIBRATION:
        case NONE:
          break;  
      }
    }
    if (targetPosition.isEmpty() || targetPosition.getLast() != position) {
      targetPosition.add(position);
    }

    // Show what position the arm is moving to
    setArmColor(position, Colors.white);
  }

  public void setArmColor(Position position, LED.Color color) {
    switch (position) {
      case GRID_LOW:
        s_Led.setColor(color, 0.0, 20.0);
        break;
      case GRID_MID:
        s_Led.setColor(color, 0.0, 50.0);
        break;
      case INTAKE_SUBSTATION:
        s_Led.setColor(color, 0.0, 65.0);
        break;
      case GRID_HIGH:
        s_Led.setColor(color, 0.0, 80.0);
        break;
      default:
        s_Intake.setColor();
        break;
    }
  }

  public void beginMovement() {
    Position nextPositionType = targetPosition.peek();
    ArmPosition nextPosition;

    // If moving to chassis and holding a cone, tuck a slightly tighter position
    if (nextPositionType == Position.CHASSIS && DriverStation.isTeleopEnabled() && !s_Intake.isCube() && s_Intake.isStalled()) {
      nextPositionType = Position.CHASSIS_CONE;
    }

    // TODO Consider passing a State, so that velocity can be non-zero for intermediate points
    nextPosition = armPositions.get(nextPositionType);
    lowerPidController.setGoal(nextPosition.lowerArmAngle);
    upperPidController.setGoal(nextPosition.upperArmAngle);
    if (DriverStation.isTeleop()) {
      if (!s_Intake.isCube() && s_Intake.isStalled()) {
        upperPidController.setPID(upperConstantsLaden.kP, upperConstantsLaden.kI, upperConstantsLaden.kD);
        lowerPidController.setPID(lowerConstantsLaden.kP, lowerConstantsLaden.kI, lowerConstantsLaden.kD);
      } else {
        upperPidController.setPID(upperConstantsTeleOp.kP, upperConstantsTeleOp.kI, upperConstantsTeleOp.kD);
        lowerPidController.setPID(lowerConstants.kP, lowerConstants.kI, lowerConstants.kD);
      }
    }
    if (targetPosition.size() > 1) {
      if (targetPosition.peek() == Position.SAFE) {
        lowerPidController.setTolerance(10);
        upperPidController.setTolerance(10);
      } else {
        lowerPidController.setTolerance(5, 15);
        upperPidController.setTolerance(5, 15);
      }
    } else {
      if (targetPosition.peek() == Position.INTAKE_SUBSTATION) {
        lowerPidController.setTolerance(1, 3);
        upperPidController.setTolerance(1, 3);
      } else {
        lowerPidController.setTolerance(2, 5);
        upperPidController.setTolerance(2, 5);
      }
    }
    lowerPidController.reset(getLowerAngle());
    upperPidController.reset(getUpperAngle());
  }

  public void updateMovement() {
    if (lowerPidController.atGoal() && upperPidController.atGoal()) {
      lastPosition = targetPosition.remove();
      if (!doneMovement()) {
        beginMovement();
      }
    }
    if (!doneMovement()) {
      if (lowerPidController.atGoal()) {
        lowerArmMaster.stopMotor();
      } else {
        lowerArmMaster.setVoltage(constrainValue(lowerPidController.calculate(getLowerAngle()), lowerMaxVoltage));
      }
      if (upperPidController.atGoal()) {
        upperArmMaster.stopMotor();
      } else {
        upperArmMaster.setVoltage(constrainValue(upperPidController.calculate(getUpperAngle()), upperMaxVoltage));
      }
    } else {
      stopMovement();
      setArmColor(lastPosition, Colors.blue);
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
      SmartDashboard.putNumber("Lower Arm goal", lowerPidController.getGoal().position);
      SmartDashboard.putNumber("Upper Arm goal", upperPidController.getGoal().position);
      SmartDashboard.putBoolean("Lower Arm set? ", lowerPidController.atGoal());
      SmartDashboard.putBoolean("Upper Arm set? ", upperPidController.atGoal());

      SmartDashboard.putNumber("Lower Angle ABSOLUTE", getLowerAbsoluteAngle());
      SmartDashboard.putNumber("Upper Angle ABSOLUTE", getUpperAbsoluteAngle());
      SmartDashboard.putNumber("Lower Angle", getLowerAngle());
      SmartDashboard.putNumber("Upper Angle", getUpperAngle());

      SmartDashboard.putNumber("Lower Arm Master Voltage:", lowerArmMaster.getMotorOutputVoltage());
      SmartDashboard.putNumber("Lower Arm Slave Voltage:", lowerArmSlave.getMotorOutputVoltage());

      SmartDashboard.putNumber("Upper Arm Master Voltage:", upperArmMaster.getMotorOutputVoltage());
      SmartDashboard.putNumber("Upper Arm Slave Voltage:", upperArmSlave.getMotorOutputVoltage());

      SmartDashboard.putNumber("Upper Arm P-Value: ", upperPidController.getP());
      SmartDashboard.putNumber("Lower Arm P-Value: ", lowerPidController.getP());
    }
  }
}