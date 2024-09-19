package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WristSubsystem extends SubsystemBase
{
    private static final double WRIST_DEPOSIT = 1;
    private static final double WRIST_INTAKE = 1.0;
    private static final double WRIST_STOW = 0;
    private static final double WRIST_SAMPLE = 1;
    private static final double WRISTURN_DEPOSIT = 0;
    private static final double WRISTURN_INTAKE = 0;
    private static final double WRISTURN_STOW = 0;
    private static final double WRISTURN_SAMPLE = 0.5;


    private static Servo wristServo;
    private static Servo wristurnServo;

    public WristSubsystem(final HardwareMap hMap) {
        wristServo = hMap.get(Servo.class, "wrist");
        wristurnServo = hMap.get(Servo.class, "wristurn");

    }

    public void setWristIntake() {
        wristServo.setPosition(WRIST_INTAKE);
        wristurnServo.setPosition(WRISTURN_INTAKE);

    }

    public void setWristDeposit() {
        wristServo.setPosition(WRIST_DEPOSIT);
        wristurnServo.setPosition(WRISTURN_DEPOSIT);

    }

    public void setWristStow() {
        wristServo.setPosition(WRIST_STOW);
        wristurnServo.setPosition(WRISTURN_STOW);
    }

    public void setWristSample() {
        wristServo.setPosition(WRIST_SAMPLE);
        wristurnServo.setPosition(WRISTURN_SAMPLE);
    }
}
