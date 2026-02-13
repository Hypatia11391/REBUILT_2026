package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
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

    private static final double MAX_SPEED = 1.0;

    // TODO: edit all the CAN IDs to proper ones
    private static final int INTAKE_ROLL_CIS_ID = 0; 
    private static final int INTAKE_LIFT_NON_CIS_ID = 0;

    private final SparkMax intakeRollCis;
    private final SparkMax intakeLiftNonCis;


    public Intake(){
        
    }
}
