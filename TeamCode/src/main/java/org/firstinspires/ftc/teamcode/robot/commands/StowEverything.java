package org.firstinspires.ftc.teamcode.robot.commands;

import static org.firstinspires.ftc.teamcode.robot.commands.extension.ExtensionGoToPosition.STOW_POSITION;
import static org.firstinspires.ftc.teamcode.robot.commands.tilt.TiltGoToPosition.TELEOP_INTAKE;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.commands.extension.ExtensionGoToPosition;
import org.firstinspires.ftc.teamcode.robot.commands.tilt.TiltGoToPosition;
import org.firstinspires.ftc.teamcode.robot.commands.wrist.WristStow;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.TiltSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.WristSubsystem;

public class StowEverything extends ParallelCommandGroup {
    public StowEverything(
            TiltSubsystem tiltSubsystem, ExtensionSubsystem extensionSubsystem,
            IntakeSubsystem clawSubsystem, WristSubsystem wristSubsystem)
    {
        addCommands(
                new SequentialCommandGroup(
                        new ExtensionGoToPosition(extensionSubsystem, STOW_POSITION),
                        new TiltGoToPosition(tiltSubsystem, TELEOP_INTAKE)
                ),
                new WristStow(wristSubsystem)
        );
        addRequirements(tiltSubsystem, clawSubsystem, wristSubsystem, extensionSubsystem);
    }
}
