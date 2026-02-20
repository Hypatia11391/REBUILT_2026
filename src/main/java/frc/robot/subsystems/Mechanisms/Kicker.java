package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/* 
 * KICKER
 *      brushed CIM x1
*/

public class Kicker extends SubsystemBase {

    private static final int KICKER_CIM_ID = 0;

    private final SparkMax kickerCim;

    public Kicker(){
        // kicker motor
        kickerCim = new SparkMax(KICKER_CIM_ID, SparkLowLevel.MotorType.kBrushed);

        configureKickerMotor(kickerCim, true);
    }
    
    public void setKickerSpeed(double speed){
        kickerCim.set(speed);
    }

    public void configureKickerMotor(SparkMax motor, boolean isInverted){
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
        kickerCim.stopMotor();
    }
    
}
