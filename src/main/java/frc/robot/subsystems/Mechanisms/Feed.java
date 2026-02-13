package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Feed extends SubsystemBase {

    private static final double MAX_SPEED = 1.0;

    private static final int FEED_CIS_ID = 0;

    private final SparkMax feedCis;

    public Feed(){
        // kicker motor
        feedCis = new SparkMax(FEED_CIS_ID, SparkLowLevel.MotorType.kBrushed);

        configureFeedMotor(feedCis, false);
    }
    
    public void configureFeedMotor(SparkMax motor, boolean isInverted){
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
    
}
