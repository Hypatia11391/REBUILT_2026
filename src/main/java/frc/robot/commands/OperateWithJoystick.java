package frc.robot.commands;

import frc.robot.subsystems.Mechanisms.Shooter;
import frc.robot.subsystems.Mechanisms.Intake;
import frc.robot.subsystems.Mechanisms.Kicker;
import frc.robot.subsystems.Mechanisms.Feed;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class OperateWithJoystick extends Command{
    private final Joystick stick;
    private final Shooter shooter;
    private final Intake intake;
    private final Kicker kicker;
    private final Feed feed;
    private static final double HIGH_LEFT_RPM = 3000; // A is one motor TODO: make 1500
    private static final double HIGH_RIGHT_RPM = 3000; // B is another motor TODO: make 1500

    private static final double KICKER_PWR = 0.7;
    private static final double FEED_PWR = 0.7;

    private static final double INTAKE_LIFT_PWR_UP = 0.35;
    private static final double INTAKE_LIFT_PWR_DOWN = -0.20;
    public static final double INTAKE_FEED_PWR = 0.3;

    public static double test = 1;

    private double atSpeedSince = -1.0;

    enum IntakeFeedState {OFF, FWD, REV};
    private IntakeFeedState intakeFeedState = IntakeFeedState.OFF; 

    enum IntakeLiftState {OFF, UP, DOWN};
    private IntakeLiftState intakeLiftState = IntakeLiftState.OFF;



      // private double spinupStartTime = 0;


    public OperateWithJoystick(Shooter shooter, Joystick stick, Intake intake, Kicker kicker, Feed feed){
        this.stick = stick;
        this.shooter = shooter;
        this.intake = intake;
        this.kicker = kicker;
        this.feed = feed;
        addRequirements(shooter, intake, kicker, feed);
    }

    

    @Override
    public void initialize(){
        shooter.stop();
        intake.stop();
        kicker.stop();
        feed.stop();
    }

    @Override
    public void execute(){

        // intake lift motor
        boolean aDOWN = stick.getRawButtonPressed(Buttons.A.ordinal()+1); 
        boolean yUP = stick.getRawButtonPressed(Buttons.Y.ordinal()+1);
        
        if (yUP){
            if (intakeLiftState == IntakeLiftState.UP){
                intakeLiftState = IntakeLiftState.OFF;
            }else{
                intakeLiftState = IntakeLiftState.UP;
            }
        }

        if (aDOWN){
            if (intakeLiftState == IntakeLiftState.DOWN){
                intakeLiftState = IntakeLiftState.OFF;
            }else{
                intakeLiftState = IntakeLiftState.DOWN;
            }
        }
        switch (intakeLiftState){
            case UP:
                intake.setLiftMotorSpeed(INTAKE_LIFT_PWR_UP);
                break;
            case DOWN:
                intake.setLiftMotorSpeed(INTAKE_LIFT_PWR_DOWN);
                break;
            case OFF:
                intake.setLiftMotorSpeed(0);
                break;
        }

        // intake feed
        boolean rb = stick.getRawButtonPressed(Buttons.RB.ordinal()+1); // intake in
        boolean lb = stick.getRawButtonPressed(Buttons.LB.ordinal()+1); // intake out

        if (rb){
            if (intakeFeedState == IntakeFeedState.FWD){
                intakeFeedState = IntakeFeedState.OFF;
            }else{
                intakeFeedState = IntakeFeedState.FWD;
            }
        }

        if (lb){
            if (intakeFeedState == IntakeFeedState.REV){
                intakeFeedState = IntakeFeedState.OFF;
            }else{
                intakeFeedState = IntakeFeedState.REV;
            }
        }

        switch (intakeFeedState) {
            case FWD:
                intake.setFeedMotorSpeed(INTAKE_FEED_PWR);
                break;
            case REV:
                intake.setFeedMotorSpeed(-INTAKE_FEED_PWR);
                break;
            case OFF:
                intake.setFeedMotorSpeed(0);
                break;
        }
        
        // shooter + delayed feed & kicker
        double rtSHOOT = applyDeadband(stick.getRawAxis(JoystickAxes.RT.ordinal()), 0.08);

        SmartDashboard.putNumber("Joystick/RT", stick.getRawAxis(JoystickAxes.RT.ordinal()));
        
        double rightTarget = rtSHOOT * HIGH_RIGHT_RPM;
        double leftTarget = rtSHOOT * HIGH_LEFT_RPM;

        // if (Autos.shooterAuto && Autos.visionOnline) {

        //     float shooterRadius = 0.05F;
        //     double thing = Aim.exitVelocity;
        //     double radiansPerSecond = thing/shooterRadius;
        //     double RPMtoShoot = (radiansPerSecond/(2*Math.PI))*60;
        //     RPMtoShoot = Math.min(RPMtoShoot, HIGH_RIGHT_RPM);                
        //     shooter.setTargetRPM(RPMtoShoot, RPMtoShoot);
        //     kicker.setKickerSpeed(KICKER_PWR);

        // }
        // else if (Autos.shooterAuto) {
            
        //     double RPMtoShoot = 3000;
        //     shooter.setTargetRPM(RPMtoShoot, HIGH_RIGHT_RPM);
        //     kicker.setKickerSpeed(KICKER_PWR);
        // }

        if (rtSHOOT != 0.0){
            // System.out.println("RT pressed, this thing should shoot!!!!!!!!!!!!!!");
            if (Aim.automaticAimControl) {
                float shooterRadius = 0.05F;
                double thing = Aim.exitVelocity;
                double radiansPerSecond = thing/shooterRadius;
                double RPMtoShoot = (radiansPerSecond/(2*Math.PI))*60;
                RPMtoShoot = Math.min(RPMtoShoot, HIGH_RIGHT_RPM);
                shooter.setTargetRPM(RPMtoShoot, RPMtoShoot);
                kicker.setKickerSpeed(KICKER_PWR);

            }
            else {
            kicker.setKickerSpeed(KICKER_PWR);
            shooter.setTargetRPM(rightTarget, leftTarget);
            // feed.setFeedSpeed(FEED_PWR);
            }

            boolean ready = shooter.atSpeed();
        
            if (ready){
                if(atSpeedSince < 0.0) atSpeedSince = Timer.getFPGATimestamp();
        }else{
            atSpeedSince = -1.0;
        }

        boolean feedAllowed = (atSpeedSince >= 0.0) && (Timer.getFPGATimestamp() - atSpeedSince >= 0.20);

        if (feedAllowed){
            feed.setFeedSpeed(FEED_PWR);
        }else{
            // kicker.stop(); // lol it was just this line
            feed.stop();
        }
        }else{
            shooter.stop();
            feed.stop();
            kicker.stop();
        }

    }

    private static double applyDeadband(double x, double db){
        return (Math.abs(x) < db) ? 0.0 : x;
    }


    @Override
    public void end(boolean interrupted){
        shooter.stop();
        kicker.stop();
        intake.stop();
        feed.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
