package org.firstinspires.ftc.teamcode.drive.Auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive; //Change this to SampleMecanumDrive.

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue_LEFT", group = "BLUE")
public class BlueLeft extends LinearOpMode {
    OpenCvCamera webcam;
    GamePropLeft.gamePropPosition propPosition = GamePropLeft.gamePropPosition.LEFT;
    SampleMecanumDrive robot;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Start OpMode", "BLUE RIGHT");
        telemetry.update();
        propPosition = startCamera();
        telemetry.addData("Selected Starting Position", propPosition);
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Identified Prop Location", propPosition);
            telemetry.update();
        }

        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            webcam.stopStreaming(); //Stop Webcam to preserve Controlhub cycles.
            runAutonoumousMode();
        }
    }

    public void runAutonoumousMode() {

        Pose2d initPose = new Pose2d(15, 65, -90); // Starting Pose --Update CoOrdinates
        Pose2d yellowPixelPose = new Pose2d(15, 65, -90);
        Pose2d purplePixelPose =new Pose2d(15, 65, -90);;
        switch (propPosition) { //UPDATE THESE POSITIONS
            case LEFT:
                telemetry.addData("Identified Prop Location", propPosition);
                yellowPixelPose = new Pose2d(23, 45, -90);
                purplePixelPose = new Pose2d(57, 44, 0);
            case CENTER:
                telemetry.addData("Identified Prop Location", propPosition);
                yellowPixelPose = new Pose2d(15, 32, -90);
                purplePixelPose = new Pose2d(57, 38, 0);
            case RIGHT:
                telemetry.addData("Identified Prop Location", propPosition);
                yellowPixelPose = new Pose2d(12, 35, 90);
                purplePixelPose = new Pose2d(57, 32, 0);

        }
        telemetry.update();
        Pose2d parkingPose = new Pose2d(57, 38, 0); //UPDATE


        robot = new SampleMecanumDrive(hardwareMap);
        robot.followTrajectorySequence(robot.trajectorySequenceBuilder(initPose) //Starting Pose
                .lineToLinearHeading(yellowPixelPose) //Drop Yellow Pixel
                .waitSeconds (5.0) //Wait for Something
                .lineToLinearHeading(parkingPose) //Parking in the back
                .build());
    }
    public GamePropLeft.gamePropPosition startCamera() {
        //Initialize Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        webcam.setPipeline(new GamePropLeft());
        return GamePropLeft.position;
    }
}