package org.firstinspires.ftc.teamcode.robot.commands.plane_launcher;

import com.arcrobotics.ftclib.command.CommandBase;

public class LaunchPlane extends CommandBase {
    private final PlaneLauncherSubsystem planeLauncherSubsystem;
    public LaunchPlane(PlaneLauncherSubsystem subsystem)
    {
        planeLauncherSubsystem = subsystem;
        addRequirements(planeLauncherSubsystem);
    }

    @Override
    public void initialize() {
        planeLauncherSubsystem.shoot();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
