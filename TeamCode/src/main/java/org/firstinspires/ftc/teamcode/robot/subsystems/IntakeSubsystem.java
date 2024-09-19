package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo


public class IntakeSubsystem extends SubsystemBase
{

    public static final double DETECTION_DISTANCE = 15;

    private static CRServo intake;
    private static DistanceSensor dist;
    private static ColorSensor color;

    public IntakeSubsystem(final HardwareMap hMap) {
        intake = hMap.get(CRServo.class, "intake");
        dist = hMap.get(DistanceSensor.class, "color");
        color=hMap.get(ColorSensor.class, "color");
    }

    public void intake() {
        intake.setPower(1);
    }
    public void stop() {
        intake.setPower(0);
    }

    public void outtake() {
        intake.setPower(-1);
    }

    public boolean detected()
    {
        return (dist.getDistance(MM)<15);
    }
    public String colorSeen(){
        String s="";
        int r= color.red();
        int b=color.blue();
        int g=color.green();
        if(r>200&&g>200) s="yellow";
        else if(r>200)s="red";
        else s="blue";
        return s;
    }
}