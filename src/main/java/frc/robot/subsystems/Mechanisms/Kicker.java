package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Kicker extends SubsystemBase {

    private static final int KICKER_CIS_ID = 0;

    private final SparkMax kickerCis;

    public Kicker(){
        // kicker motor
        kickerCis = new SparkMax(KICKER_CIS_ID, SparkLowLevel.MotorType.kBrushed);

        configureKickerMotor(kickerCis, false);
    }
    
    public void configureKickerMotor(SparkMax motor, boolean isInverted){
      SparkMaxConfig config = new SparkMaxConfig();
      config.inverted(isInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30)
            .voltageCompensation(12)
            .openLoopRampRate(0.1);

      motor.configureAsync(
        config, 
        ResetMode.kNoResetSafeParameters, 
        PersistMode.kPersistParameters);
    }

    public void stop() {
        kickerCis.stopMotor();
    }
    
}
