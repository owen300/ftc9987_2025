package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.commands.extension.ExtensionGoToPosition;

@Config
public class ExtensionSubsystem extends SubsystemBase
{
    public static enum State{
        deposit,
        intake
}
    public static State state=State.deposit;
    public static int LOW_POSISTION=0;
    Telemetry telemetry;

    public static int UNEXTENDED_POSITION(){
        if(state.equals(State.deposit)) return -56;
        return 0;
    }
    public static final int BACKBOARD_POSITION_INCREMENT = 20;

    private static double kP = 0.0058, kI = 0.0, kD = 0.0, kF = 0.0;

    public static double TOLERANCE_PID = 10;
    // tolerance where pid is calculated in ticks
    public static double ACCEPTABLE_POSITION_TOLERANCE = 20;
    // acceptable position tolerance is the tolerance for the position to be considered "at position"

    private PIDFController pidf;

    private Motor extension_top;
    private Motor extension_bottom;
    private MotorGroup extension;
    private double target=0;

    public ExtensionSubsystem(HardwareMap hMap, Telemetry telemetry)
    {
        pidf = new PIDFController(kP, kI, kD, kF);
        pidf.setTolerance(TOLERANCE_PID);
        extension_top =  new Motor(hMap, "extension_motor_1");
        extension_bottom =  new Motor(hMap, "extension_motor_2");
        extension_bottom.setInverted(true);
        extension_top.setInverted(true);
        //extension_bottom.resetEncoder();
        extension = new MotorGroup(extension_top, extension_bottom);

        this.telemetry = telemetry;

        extension.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extension.setRunMode(Motor.RunMode.RawPower);

        //extension.getCurrentPosition();
    }
    public void init(){
        extension_bottom.resetEncoder();
    }

    public int getCurrentPosition() {
        return extension_bottom.getCurrentPosition();
    }

    /**
     * asynchronous pidf goToPosition
     *
     * @param targetPosition Target Position
     */
    public void goToPosition(int targetPosition)
    {
        pidf.setSetPoint(targetPosition);
        target=targetPosition;
        if(targetPosition==ExtensionGoToPosition.HIGH_BUCKET_POS||targetPosition==ExtensionGoToPosition.LOW_BUCKET_POS||targetPosition==ExtensionGoToPosition.SAMPLE){
            state=State.deposit;
        } else if(targetPosition==ExtensionGoToPosition.INTAKE||targetPosition==ExtensionGoToPosition.INTAKE_FAR){
            state=State.intake;
        }
    }
    public void incrementUp(){
        if(extension_bottom.getCurrentPosition()+50<=ExtensionGoToPosition.ONE_STAGE_EXTENSION) {
            pidf.setSetPoint(extension_bottom.getCurrentPosition() + 50);
            target=pidf.getSetPoint();
        }else{
            pidf.setSetPoint(ExtensionGoToPosition.ONE_STAGE_EXTENSION);
            target=ExtensionGoToPosition.ONE_STAGE_EXTENSION;
        }
    }
    public void incrementDown(){
        if(extension_bottom.getCurrentPosition()-50>=UNEXTENDED_POSITION()) {
            pidf.setSetPoint(extension_bottom.getCurrentPosition() - 50);
            target=pidf.getSetPoint();
        }else {
            pidf.setSetPoint(UNEXTENDED_POSITION());
            target=UNEXTENDED_POSITION();
        }
    }
    // TODO: potential problem with not tracking the position of the
    //  extension whenever the motor isn't actively trying to go to a spot
    public boolean atTargetPosition ()
    {
        return abs(pidf.getSetPoint() - getCurrentPosition()) < ACCEPTABLE_POSITION_TOLERANCE;
    }

    /**
     * Power control through joystick
     *
     * @param joystick value of the joystick from -1 to 1
     */
    public void manualControl(double joystick)
    {
        if(getCurrentPosition()>UNEXTENDED_POSITION()&&getCurrentPosition()<ExtensionGoToPosition.ONE_STAGE_EXTENSION)extension.set(joystick);
    }

    public void periodic()
    {
       callTelemetry();
        if(!atTargetPosition())extension.set(pidf.calculate(getCurrentPosition(),target));
        else extension.set(0);
    }

    public void callTelemetry()
    {
        telemetry.addData("Extension Position", getCurrentPosition());
        telemetry.addData("Extension Target Position", pidf.getSetPoint());
    }
}
