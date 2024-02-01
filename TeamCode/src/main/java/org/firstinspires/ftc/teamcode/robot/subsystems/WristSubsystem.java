package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WristSubsystem extends SubsystemBase
{
    private static final double WRIST_DEPOSIT = 0.65;
    private static final double WRIST_INTAKE = 0.04;
    private static final double WRIST_STOW = 0.65;
    private static final double WRIST_TOP_DOWN_INTAKE = 0.65; // TODO: find value

    private static Servo wristServo;

    public WristSubsystem(final HardwareMap hMap) {
        wristServo = hMap.get(Servo.class, "wrist");

    }

    public void setWristIntake() {
        wristServo.setPosition(WRIST_INTAKE);

    }

    public void setWristDeposit() {
        wristServo.setPosition(WRIST_DEPOSIT);

    }

    public void setWristStow() {
        wristServo.setPosition(WRIST_STOW);
    }
}
