/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.Drivercontrol.drive.Feildcentricdrive;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.ftcLib_DLC.TriggerAnalogButton;
import org.firstinspires.ftc.teamcode.robot.commands.ScoreCommand;
import org.firstinspires.ftc.teamcode.robot.commands.intake.IntakeAutoCommand;
import org.firstinspires.ftc.teamcode.robot.commands.drivetrain.DriveFieldCentric;
import org.firstinspires.ftc.teamcode.robot.commands.extension.ExtensionGoToPosition;
import org.firstinspires.ftc.teamcode.robot.commands.tilt.TiltGoToPosition;
import org.firstinspires.ftc.teamcode.robot.commands.wrist.WristDeposit;
import org.firstinspires.ftc.teamcode.robot.commands.wrist.WristIntake;
import org.firstinspires.ftc.teamcode.robot.commands.wrist.WristSample;
import org.firstinspires.ftc.teamcode.robot.commands.wrist.WristStow;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.TiltSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.WristSubsystem;

@TeleOp(name="The Best Teleop Known To Mankind", group="Linear OpMode")

public final class TheBestTeleopKnownToMankind extends CommandOpMode
{
    public TiltSubsystem tiltSubsystem;
    @Override
    public void initialize()
    {
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);
        tiltSubsystem = new TiltSubsystem(hardwareMap,telemetry);
        WristSubsystem wristSubsystem = new WristSubsystem(hardwareMap);
        ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem(hardwareMap,telemetry);
        DriveSubsystem driveSubsystem;
        driveSubsystem = new DriveSubsystem(hardwareMap);
        ScoreCommand score=new ScoreCommand(extensionSubsystem,wristSubsystem,tiltSubsystem,intakeSubsystem);

        ParallelCommandGroup stow= new ParallelCommandGroup(
                new InstantCommand(()->CommandScheduler.getInstance().cancel(CommandScheduler.getInstance().requiring(tiltSubsystem))),
                new WristStow(wristSubsystem),
                new SequentialCommandGroup(
                        new TiltGoToPosition(tiltSubsystem, TiltGoToPosition.STOW),
                        new ExtensionGoToPosition(extensionSubsystem, ExtensionGoToPosition.STOW_POSITION)
                ));
        //driver
        TriggerAnalogButton driverTrigger =
                new TriggerAnalogButton(driver,GamepadKeys.Trigger.LEFT_TRIGGER,0.7);
        TriggerAnalogButton scoreTrigger =
                new TriggerAnalogButton(operator,GamepadKeys.Trigger.RIGHT_TRIGGER,0.7);

        driveSubsystem.setDefaultCommand(
                new DriveFieldCentric(
                driveSubsystem,
                () -> driver.getLeftY(),
                () -> driver.getLeftX(),
                () -> driver.getRightX(),
                () -> driverTrigger.get()));

        //manual extension by default unless another command using it runs



        //auto intake
        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new ParallelCommandGroup(new InstantCommand(()->CommandScheduler.getInstance().cancel(CommandScheduler.getInstance().requiring(intakeSubsystem))),
                new WristIntake(wristSubsystem),
                new SequentialCommandGroup(
                        new TiltGoToPosition(tiltSubsystem,TiltGoToPosition.TELEOP_INTAKE),
                        new ExtensionGoToPosition(extensionSubsystem, 0),
                        new IntakeAutoCommand(intakeSubsystem, Alliance.color),
                        stow
                        )));

        driver.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(()->driveSubsystem.resetIMU()));

        //low bucket pos
        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new ParallelCommandGroup(
                new WristDeposit(wristSubsystem),
                new SequentialCommandGroup(
                        new TiltGoToPosition(tiltSubsystem, TiltGoToPosition.TELEOP_BUCKETL),
                        new ExtensionGoToPosition(extensionSubsystem,ExtensionGoToPosition.LOW_BUCKET_POS)
                        ))).whenReleased(stow);
        //high bucket
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ParallelCommandGroup(
                        new WristDeposit(wristSubsystem),
                        new SequentialCommandGroup(
                                new TiltGoToPosition(tiltSubsystem, TiltGoToPosition.TELEOP_BUCKETH),
                                new ExtensionGoToPosition(extensionSubsystem,ExtensionGoToPosition.HIGH_BUCKET_POS)
                        ))).whenReleased(stow);
        //sample
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ParallelCommandGroup(
                        new WristSample(wristSubsystem),
                        new SequentialCommandGroup(
                                new TiltGoToPosition(tiltSubsystem, TiltGoToPosition.TELEOP_SAMPLE),
                                new ExtensionGoToPosition(extensionSubsystem,ExtensionGoToPosition.SAMPLE)
                        ))).whenReleased(stow);
        //stow
        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(stow);

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).and(scoreTrigger).whenActive(score.getScoreCommand(GamepadKeys.Button.DPAD_DOWN));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).and(scoreTrigger).whenActive(score.getScoreCommand(GamepadKeys.Button.DPAD_UP));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).and(scoreTrigger).whenActive(score.getScoreCommand(GamepadKeys.Button.DPAD_RIGHT));

        // should be able to get interrupted by ExtensionGoToPosition
        //CommandScheduler.getInstance().schedule(true,extendoManualCommand);

        extensionSubsystem.init();
        tiltSubsystem.init();
        while(opModeInInit()){
            if(gamepad1.circle){
                driveSubsystem.init();

            }
        }

    }

    @Override
    public void run()
    {
        super.run();

        telemetry.update();
    }
}


