package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public TalonFXConfiguration armMotorConfig;
    public static CANCoderConfiguration armCanCoderConfig;


    public CTREConfigs(){
        swerveCanCoderConfig = new CANCoderConfiguration();

        armMotorConfig = new TalonFXConfiguration();
        armCanCoderConfig = new CANCoderConfiguration();
    
        /* Arm Motor Configurations */

        

        armMotorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        
        /* Arm CANCoder Configuration */
        armCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        armCanCoderConfig.sensorDirection = false;
        armCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        armCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        

    }

}
