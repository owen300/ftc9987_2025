package org.firstinspires.ftc.teamcode.opmode.auto.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.ftcLib_DLC.AutoUtil;
import org.firstinspires.ftc.teamcode.robot.commands.claw.ClawCloseCommand;
import org.firstinspires.ftc.teamcode.robot.commands.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.robot.commands.extension.ExtensionGoToPosition;
import org.firstinspires.ftc.teamcode.robot.commands.tilt.TiltGoToPosition;
import org.firstinspires.ftc.teamcode.robot.commands.wrist.WristDeposit;
import org.firstinspires.ftc.teamcode.robot.commands.wrist.WristIntake;
import org.firstinspires.ftc.teamcode.robot.commands.wrist.WristStow;
import org.firstinspires.ftc.teamcode.robot.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.TiltSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.vision.TeamElementPipeline;
import org.firstinspires.ftc.teamcode.vision.Vision;

@Autonomous(group="auto", name="Auto_redright_side")
public class AutoRedRight extends LinearOpMode
{
    private static final double TILE = 24.0;
    @Override
    public void runOpMode() throws InterruptedException {
        AutoUtil util= new AutoUtil();
        AutoMecanumDrive drive = new AutoMecanumDrive(hardwareMap);
        DriveSubsystem driveSubsystem=new DriveSubsystem(hardwareMap);
        ClawSubsystem clawSubsystem = new ClawSubsystem(hardwareMap);
        TiltSubsystem tiltSubsystem = new TiltSubsystem(hardwareMap, telemetry);
        WristSubsystem wristSubsystem = new WristSubsystem(hardwareMap);
        ExtensionSubsystem extensionSubsystem=new ExtensionSubsystem(hardwareMap,telemetry);
        Pose2d startPose =  new Pose2d(TILE*0.6, -2.5*TILE, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        ParallelCommandGroup initiate = new ParallelCommandGroup(
                new ClawCloseCommand(clawSubsystem),
                new WristStow(wristSubsystem),
                new InstantCommand(()->{
                    driveSubsystem.init();
                    tiltSubsystem.init();
                    extensionSubsystem.init();
                })
        );
        SequentialCommandGroup place_pixel_and_stow = new SequentialCommandGroup(
                new TiltGoToPosition(tiltSubsystem, TiltGoToPosition.AUTO_STACK_INTAKE1),
                new WristIntake(wristSubsystem),
                new ClawOpenCommand(clawSubsystem, ClawOpenCommand.Side.RIGHT),
                new WristStow(wristSubsystem),
                new TiltGoToPosition(tiltSubsystem, TiltGoToPosition.TELEOP_INTAKE));

        SequentialCommandGroup deposit = new SequentialCommandGroup(
        new TiltGoToPosition(tiltSubsystem, TiltGoToPosition.TELEOP_DEPOSIT),
                new WristDeposit(wristSubsystem),
                new ClawOpenCommand(clawSubsystem, ClawOpenCommand.Side.BOTH),
                new WristStow(wristSubsystem),
                new TiltGoToPosition(tiltSubsystem, TiltGoToPosition.TELEOP_INTAKE),
                new ExtensionGoToPosition(extensionSubsystem,0),
                new ClawCloseCommand(clawSubsystem));



        TrajectorySequence Center = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(TILE/2, -1.53*TILE+1))
                .turn(Math.toRadians(-170))
                .lineToLinearHeading(new Pose2d(TILE*2, -1.5*TILE, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(TILE*1.9,-2.5*TILE))
                .build();
                /////////////////


        TrajectorySequence Right = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(TILE/2, -1.4*TILE+1))
                .turn(Math.toRadians(-110))
                .lineToLinearHeading(new Pose2d(TILE*2, -1.25*TILE, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(TILE*1.9,-2.5*TILE))
                .build();

        TrajectorySequence Left = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(TILE/2, -1.4*TILE+1))
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(TILE*1.3,-1.4*TILE))
                .lineToLinearHeading(new Pose2d(TILE*2, -1.75*TILE, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(TILE*1.9,-2.5*TILE))
                .build();



        TeamElementPipeline.MarkerPosistion markerPosistion;
        Vision.startStreaming(hardwareMap, telemetry);
        markerPosistion = TeamElementPipeline.MarkerPosistion.CENTER;
        CommandScheduler.getInstance().schedule(initiate);
        CommandScheduler.getInstance().run();
        while(opModeInInit()) {
            markerPosistion = Vision.determineMarkerPosistion();
        }
        ////////////////////////////////////////////


        waitForStart();
        Vision.webcam.stopStreaming();
        switch (markerPosistion) {
            case CENTER:
            case UNKNOWN:
                drive.followTrajectorySequenceAsync(Center);
                break;
            case RIGHT:
                drive.followTrajectorySequenceAsync(Right);
                break;
            case LEFT:
                drive.followTrajectorySequenceAsync(Left);
                break;
        }
        /*telemetry.addData("DONE 2",0);
        telemetry.update();*/
        while(!isStopRequested()) {
            drive.update();
            CommandScheduler.getInstance().run();


        }
    }

}