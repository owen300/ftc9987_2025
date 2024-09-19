package org.firstinspires.ftc.teamcode.robot.commands.tilt;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.TiltSubsystem;

/**
 * open claw
 */
public class TiltGoToPosition extends CommandBase
{
    public static double TELEOP_INTAKE = 0; // floor intake
    public static double STOW=0;
    public static double TELEOP_BUCKETL = 80;
    public static double TELEOP_BUCKETH = 80;
    public static double TELEOP_SAMPLE = 50;
    private final TiltSubsystem tiltSubsystem;
    private final double targetAngle;

    public TiltGoToPosition(TiltSubsystem subsystem, double targetAngle)
    {
        tiltSubsystem = subsystem;
        this.targetAngle = targetAngle;
        addRequirements(tiltSubsystem);
    }

    @Override
    public void initialize()
    {
        tiltSubsystem.setTargetAngle(targetAngle);
    }

    @Override
    public boolean isFinished()
    {
        return tiltSubsystem.atTargetPosition();
    }
}
