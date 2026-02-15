package frc.robot.commands;

import frc.robot.subsystems.Mechanisms.Shooter;
import frc.robot.subsystems.Mechanisms.Intake;
import frc.robot.subsystems.Mechanisms.Kicker;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;


public class OperateWithJoystick extends Command{
    private final Joystick stick;
    private final Shooter shooter;
    private final Intake intake;
    private final Kicker kicker;


    private static final double HIGH_TOP_RPM = 1500;
    private static final double HIGH_BOTTOM_RPM = 1500;


    // private static final double KICKER_FEED_PWR = 0.7;
    // private static final double INTAKE_FEED_PWR = 0.4;

      // private double spinupStartTime = 0;


    public OperateWithJoystick(Shooter shooter, Joystick stick, Intake intake, Kicker kicker){
        this.stick = stick;
        this.shooter = shooter;
        this.intake = intake;
        this.kicker = kicker;
        addRequirements(shooter, intake, kicker);
    }

    

    @Override
    public void initialize(){
        shooter.stop();
        intake.stop();
        kicker.stop();
    }

    @Override
    public void execute(){
        // boolean rtHeld = stick.getRawButton(Buttons.RT.ordinal());

        double rt = stick.getRawAxis(JoystickAxes.RT.ordinal());
        rt = applyDeadband(rt, 0.08);

        double topTarget = rt * HIGH_TOP_RPM;
        double bottomTarget = rt * HIGH_BOTTOM_RPM;

        // boolean lowMode = stick.getRawButton(Buttons.RB.ordinal());

        boolean reverseHELD = stick.getRawButton(Buttons.LT.ordinal()); // TODO: recode through JoystickAxes.LT.ordinal()

        if (reverseHELD){
            shooter.stop();
            // kicker.set(-0.4);
            // intake.set(-0.4);
            return;
        }

        if (rt > 0.08){
            shooter.setTargetRPM(topTarget, bottomTarget);
        
            if (shooter.atSpeed()){
            // kicker.set(KICKER_FEED_PWR); // TODO: make a setter for kicker
        }else{
            kicker.stop();
        }
        }else{
            shooter.stop();
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
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
