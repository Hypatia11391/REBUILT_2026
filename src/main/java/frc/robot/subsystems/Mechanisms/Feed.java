package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/* 
 * FEED
 *      brushless NEO x1
*/

public class Feed extends SubsystemBase {
    private static final int FEED_NEO_ID = 0;

    private final SparkMax feedNeo;

    public Feed(){
        // feed motor
        feedNeo = new SparkMax(FEED_NEO_ID, SparkLowLevel.MotorType.kBrushless);

        configureFeedMotor(feedNeo, false);
    }
    
    public void setFeedSpeed(double speed){
        feedNeo.set(speed);
    }

    public void configureFeedMotor(SparkMax motor, boolean isInverted){
      SparkMaxConfig config = new SparkMaxConfig();
      config.inverted(isInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .voltageCompensation(12)
            .openLoopRampRate(0.1);

      motor.configureAsync(
        config, 
        ResetMode.kNoResetSafeParameters, 
        PersistMode.kPersistParameters);
    }

    public void stop() {
        feedNeo.stopMotor();
    }
    
}
