package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.robot.commands.extension.ExtensionGoToPosition;
import org.firstinspires.ftc.teamcode.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.TiltSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.WristSubsystem;

public class ScoreCommand {
    private final int samplePullDown=0;//adjust to whatever fully scored a sample
    private final ExtensionSubsystem extendo;
    private final WristSubsystem wrist;
    private final TiltSubsystem tilt;
    private final IntakeSubsystem intake;
    public ScoreCommand(ExtensionSubsystem extendo, WristSubsystem wrist, TiltSubsystem tilt, IntakeSubsystem intake){
        this.extendo=extendo;
        this.tilt=tilt;
        this.wrist=wrist;
        this.intake=intake;
    }
    private SequentialCommandGroup scoreSample(){
        return new SequentialCommandGroup(
                new ExtensionGoToPosition(extendo, ExtensionGoToPosition.SAMPLE-samplePullDown),
                new InstantCommand(()-> intake.outtake())
        );
    }
    private SequentialCommandGroup scoreBucket(){
        return new SequentialCommandGroup(
                new InstantCommand(()-> intake.outtake())
        );
    }
    public SequentialCommandGroup getScoreCommand(GamepadKeys.Button button){
        if(button==GamepadKeys.Button.DPAD_UP)return scoreBucket();
        else if(button==GamepadKeys.Button.DPAD_RIGHT)return scoreBucket();
        else return scoreSample();
    }

}
