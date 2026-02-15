package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


  /*  
  *   brushed - CIS for the rollers
  *   brushed - non CIS lifting
  *   brushed - CIS for the kicker
  *   brushless - NEO x2
  */


public class Intake extends SubsystemBase {

    // TODO: edit all the CAN IDs to proper ones
    private static final int INTAKE_ROLL_CIS_ID = 0; 
    private static final int INTAKE_LIFT_NON_CIS_ID = 0;

    private final SparkMax intakeRollCis;
    private final SparkMax intakeLiftNonCis;    

    public Intake(){
      // intake motors
      intakeRollCis = new SparkMax(INTAKE_ROLL_CIS_ID, SparkLowLevel.MotorType.kBrushed);
      intakeLiftNonCis = new SparkMax(INTAKE_LIFT_NON_CIS_ID, SparkLowLevel.MotorType.kBrushed);

      configureIntakeMotor(intakeRollCis, false);
      configureIntakeLiftMotor(intakeLiftNonCis, false);

    }

    public void configureIntakeLiftMotor(SparkMax motor, boolean isInverted){
      SparkMaxConfig config = new SparkMaxConfig();
      config.inverted(isInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(25)
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
            .smartCurrentLimit(50)
            .voltageCompensation(12)
            .openLoopRampRate(0.1);
      motor.configureAsync(
        config, 
        ResetMode.kNoResetSafeParameters, 
        PersistMode.kPersistParameters);
    }

    public void stop() {
        intakeLiftNonCis.stopMotor();
        intakeRollCis.stopMotor();
    }
}
