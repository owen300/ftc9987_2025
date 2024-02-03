package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HangSubsystem extends SubsystemBase {
    private DcMotorEx hang_motor;
    private boolean forward=false;
    private boolean backward=false;
    public HangSubsystem(HardwareMap hardwareMap){
        hang_motor=hardwareMap.get(DcMotorEx.class,"hang");
        hang_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void moveforward(){
        forward=true;
    }
    public void movebackward(){
        backward=true;
    }
    @Override
    public void periodic() {
        if(forward){hang_motor.setPower(1);forward=false;}
        else if(backward){hang_motor.setPower(-1);backward=false;}
        else hang_motor.setPower(0);
    }

}
