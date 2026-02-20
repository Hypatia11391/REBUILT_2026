package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/*
* INTAKE
*   brushed CIM x1 for the rollers
*   brushed non-CIM x1 for lifting
*/ 

public class Intake extends SubsystemBase {

    // TODO: edit all the CAN IDs to proper ones
    private static final int INTAKE_FEED_ID = 0;
    private static final int INTAKE_LIFT_ID = 0;

    private static final int TOP_LIMIT_SWITCH_ID = 0;
    private static final int BOTTOM_LIMIT_SWITCH_ID = 1;

    private final DigitalInput topLimitSwitch = new DigitalInput(TOP_LIMIT_SWITCH_ID);
    private final DigitalInput bottomLimitSwitch = new DigitalInput(BOTTOM_LIMIT_SWITCH_ID);

    private final SparkMax intakeFeed;
    private final SparkMax intakeLift;    

    public Intake(){
      // intake motors
      intakeFeed = new SparkMax(INTAKE_FEED_ID, SparkLowLevel.MotorType.kBrushed);
      intakeLift = new SparkMax(INTAKE_LIFT_ID, SparkLowLevel.MotorType.kBrushed);

      configureIntakeMotor(intakeFeed, false);
      configureIntakeLiftMotor(intakeLift, false);

    }


    public void setLiftMotorSpeed(double speed){
      if (speed > 0){
        if (topLimitSwitch.get()){
          intakeLift.set(0);
        }else{
          intakeLift.set(speed);
        }
      }else{
        if (bottomLimitSwitch.get()){
          intakeLift.set(0);
        }else{
          intakeLift.set(speed);
        }
      }
    }

    public void setFeedMotorSpeed(double speed){
      intakeFeed.set(speed);
    }

    public void configureIntakeLiftMotor(SparkMax motor, boolean isInverted){
      SparkMaxConfig config = new SparkMaxConfig();
      config.inverted(isInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)
            .voltageCompensation(12)
            .openLoopRampRate(0.2);

      motor.configureAsync(
        config, 
        ResetMode.kNoResetSafeParameters, 
        PersistMode.kPersistParameters);
    }

    public void configureIntakeMotor(SparkMax motor, boolean isInverted){
      SparkMaxConfig config = new SparkMaxConfig();
      config.inverted(isInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)
            .voltageCompensation(12)
            .openLoopRampRate(0.1);
      motor.configureAsync(
        config, 
        ResetMode.kNoResetSafeParameters, 
        PersistMode.kPersistParameters);
    }

    public void stop() {
        intakeLift.stopMotor();
        intakeFeed.stopMotor();
    }
}
