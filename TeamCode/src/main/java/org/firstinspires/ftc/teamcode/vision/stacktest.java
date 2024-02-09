package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class stacktest extends OpMode {

    private StackPipeline stackPipeline;
    private VisionPortal portal;

    @Override
    public void init(){


        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessors(stackPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        portal.setProcessorEnabled(stackPipeline, true);
        FtcDashboard.getInstance().startCameraStream(stackPipeline, 0);

    }
    @Override
    public void init_loop(){
        telemetry.addData("x pos",stackPipeline.closestPixelContour.x);
        telemetry.addData("y pos",stackPipeline.closestPixelContour.y);
    }
    @Override
    public void loop(){

    }

}
