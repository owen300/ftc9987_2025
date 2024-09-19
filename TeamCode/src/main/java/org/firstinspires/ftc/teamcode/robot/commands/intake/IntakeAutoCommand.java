package org.firstinspires.ftc.teamcode.robot.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;

/**
 * open claw
 * waits till the distance sensor detects somethings
 * closes claw
 */
public class IntakeAutoCommand extends CommandBase
{
    private final IntakeSubsystem intakeSubsystem;
    private String color;
    public IntakeAutoCommand(IntakeSubsystem subsystem,String color)
    {
        intakeSubsystem = subsystem;
        this.color=color;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.intake();
    }
    @Override
    public void execute(){
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.detected()&&(intakeSubsystem.colorSeen()==color||intakeSubsystem.colorSeen()=="yellow");
    }

    @Override
    public void end(boolean interrupted)
    {
        intakeSubsystem.stop();
    }
}
