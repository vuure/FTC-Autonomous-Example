package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
@Autonomous(name="Basic: Iterative OpMode Auto", group="Iterative OpMode")
@Disabled
public class RobotAutoCodeBlueClose extends OpMode {
    private static final boolean USE_WEBCAM = true;
    //private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/pixelModel.tflite";
    private static final String[] LABELS = {
            "pixel",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor intakeMotor = null;
    private Servo planeServo = null;
    private IMU imu =null;
    private final ElapsedTime runtime = new ElapsedTime();
    private String placeOfPixel = null;
    private double encoderTick = 28;
    //dişli oranı ile çarpılacak 420
    private double lenghtOfWheel = 25.13;
    public void init() {
        initTfod();

        leftDrive  = hardwareMap.get(DcMotor.class, "motor1");
        rightDrive = hardwareMap.get(DcMotor.class, "motor2");
        intakeMotor = hardwareMap.get(DcMotor.class, "motor3");

        planeServo = hardwareMap.get(Servo.class, "servo1");

        planeServo.setPosition(0.66);

        imu = hardwareMap.get(IMU.class,"imu");

        //Bu değerleri değiştir
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init_loop() {

    }
    public void start() {

        double xOfPixel = getPixel();

        if (xOfPixel<426){
            placeOfPixel = "Left";
        } else if(426< xOfPixel && xOfPixel < 852){
            placeOfPixel = "Middle";
        } else if (852<xOfPixel) {
            placeOfPixel = "Right";
        }

        runtime.reset();
    }
    public void loop() {
        telemetry.addData("Pikselin Konumu",placeOfPixel);
        if (placeOfPixel == "Left"){
            if(runtime.seconds()<4){
                runEncoder(38);
            } else if(runtime.seconds()<8){
                turnRobot(-90);
            } else if(runtime.seconds()<12){
                runEncoder(2);
            } else if(runtime.seconds() < 16){
                runIntake(0.5);
            } else if(runtime.seconds()<20){
                turnRobot(0);
            } else if(runtime.seconds()<24){
                runEncoder(24);
            } else if (runtime.seconds()<28) {
                turnRobot(-90);
            } else if (runtime.seconds()<32) {
                runEncoder(22);
            } else if(runtime.seconds()<36){
                turnRobot(90);
            } else if(runtime.seconds() < 40){
                runEncoder(5.5);
            } else if(runtime.seconds() < 44){
                runIntake(3);
            }
        } else if(placeOfPixel == "Middle"){
            if(runtime.seconds()<4){
                runEncoder(38);
            } else if(runtime.seconds()<8){
                turnRobot(0);
            } else if(runtime.seconds()<12){
                runEncoder(2);
            } else if(runtime.seconds() < 16){
                runIntake(0.5);
            } else if(runtime.seconds()<20){
                turnRobot(-90);
            } else if(runtime.seconds()<24){
                runEncoder(22);
            } else if (runtime.seconds()<28) {
                turnRobot(-90);
            } else if (runtime.seconds()<32) {
                runEncoder(22);
            } else if(runtime.seconds()<36){
                turnRobot(90);
            } else if(runtime.seconds() < 40){
                runEncoder(5.5);
            } else if(runtime.seconds() < 44){
                runIntake(3);
            }
        } else if (placeOfPixel == "Right"){
            if(runtime.seconds()<4){
                runEncoder(38);
            } else if(runtime.seconds()<8){
                turnRobot(90);
            } else if(runtime.seconds()<12){
                runEncoder(2);
            } else if(runtime.seconds() < 16){
                runIntake(0.5);
            } else if(runtime.seconds()<20){
                turnRobot(180);
            } else if(runtime.seconds()<24){
                runEncoder(22);
            } else if (runtime.seconds()<28) {
                turnRobot(-90);
            } else if (runtime.seconds()<32) {
                runEncoder(22);
            } else if(runtime.seconds()<36){
                turnRobot(90);
            } else if(runtime.seconds() < 40){
                runEncoder(5.5);
            } else if(runtime.seconds() < 44){
                runIntake(3);
            }
        }
    }
    private void initTfod() {

        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(tfod);

        visionPortal = builder.build();

    }
    double getPixel() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        double x = 0;
        double y = 0;
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
        }

        return x;
    }
    private void runEncoder(double moveToInch){
        double desiredTour = (moveToInch/lenghtOfWheel);
        double encoderPositionSet = desiredTour*encoderTick;

        leftDrive.setTargetPosition((int)(encoderPositionSet));
        rightDrive.setTargetPosition((int)(encoderPositionSet));

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(0.6);
        rightDrive.setPower(0.6);
        while ((leftDrive.isBusy() && rightDrive.isBusy())) {
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void turnRobot(double desiredDegree){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double angle = orientation.getYaw(AngleUnit.DEGREES);

        if(desiredDegree-angle<0){
            leftDrive.setPower(-0.5);
            rightDrive.setPower(0.5);
        } else if (0<desiredDegree-angle){
            leftDrive.setPower(0.5);
            rightDrive.setPower(-0.5);
        }

    }
    private void runIntake(double desiredTour){

        double encoderPositionSet = desiredTour*encoderTick;

        intakeMotor.setTargetPosition((int)(encoderPositionSet));

        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeMotor.setPower(0.1);

        while (intakeMotor.isBusy()) {
            // Display it for the driver.
            telemetry.addData("Running to",  " %7d", encoderPositionSet);
            telemetry.addData("Currently at",  " at %7d",
                    intakeMotor.getCurrentPosition());
            telemetry.update();
        }

        intakeMotor.setPower(0);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void stop(){
        intakeMotor.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setPower(0);
    }
}
