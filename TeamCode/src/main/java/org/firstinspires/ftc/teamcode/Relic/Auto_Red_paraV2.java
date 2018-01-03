package org.firstinspires.ftc.teamcode.Relic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "Auto Red || (V2)", group = "3650 Testing")
public class Auto_Red_paraV2 extends LinearOpMode {
    private VuforiaLocalizer vuforia;
    private PrivateData priv = new PrivateData();

    private DcMotor lDrive, rDrive, lift1;
    private Servo armServo, grabber;
    private ColorSensor cSensor;

    private double initialHeading;

    @Override
    public void runOpMode() throws InterruptedException {
        double armExtendedPos = 0.0;
        double armRetractedPos = 0.75;

        IMU_class imu = new IMU_class("imu", hardwareMap);
        initialHeading = getHeading(imu);

        rDrive = hardwareMap.dcMotor.get("rDrive");
        rDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lDrive = hardwareMap.dcMotor.get("lDrive");
        lDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rDrive.setDirection(DcMotor.Direction.REVERSE);

        lift1 = hardwareMap.dcMotor.get("lift1");


        armServo = hardwareMap.servo.get("armServo");
        cSensor = hardwareMap.colorSensor.get("cSensor");

        grabber = hardwareMap.servo.get("grabber");

        // START VUFORIA

        //make camera view show up on screen
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        parameters.vuforiaLicenseKey = priv.vuforiaKey;


        //use back camera
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //Shows XYZ axes on detected object (Teapots, buildings, and none also valid)
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.TEAPOT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //Get image identification files
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        // END VUFORIA



        waitForStart();

        //Move grabber/lift
        grabber.setPosition(.8);
        Thread.sleep(500);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift1.setTargetPosition(600); //Need to change this!!
        lift1.setPower(.99);

        //Down servo
        armServo.setPosition(armExtendedPos);
        Thread.sleep(1000);

        relicTrackables.activate(); // Start Vuforia object search
        Thread.sleep(1000);

        int col = -2; //1=R, 0=C, -1=L, -2=undetected
        long startTime = System.currentTimeMillis();

        while(opModeIsActive() && (System.currentTimeMillis()-startTime)< 4000){ //Run for 4 secs
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("Key: ", vuMark);
                telemetry.update();
                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    col = 1;
                    break;
                }
                else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    col = 0;
                    break;
                }
                else if (vuMark == RelicRecoveryVuMark.LEFT) {
                    col = -1;
                    break;
                }

            }

            else{
                telemetry.addData("No key detected!", null);
            }
            telemetry.update();
            idle();
        }


        relicTrackables.deactivate(); // End Vuforia search

        if (cSensor.blue() > cSensor.red()){
            //Move back if blue (NEED TO CHECK)
            setBothEncoder(-600);
            setBothPower(.4);
            while (lDrive.isBusy()){ idle(); }
            armServo.setPosition(armRetractedPos);
        }

        // Move far forward
        setBothEncoder(2000);
        setBothPower(0.4);

        while (lDrive.isBusy()){ idle(); }
        setBothPower(0.0);
        armServo.setPosition(armRetractedPos);
        Thread.sleep(400);

        //Run backwards into stone (gets a consistent staring pos)
        setBothEncoder(-1000);
        setBothPower(0.4);

        if (col == 1){
            setBothEncoder(800);
            setBothPower(.4);
        }
        else if (col == 0){
            setBothEncoder(1500);
            setBothPower(.4);
        }
        else if (col == -1){
            setBothEncoder(2200);
            setBothPower(.4);
        }
        else {
            //Go for center
            setBothEncoder(1500);
            setBothPower(.4);
        }

        while (lDrive.isBusy()){ idle(); }

        turn2Angle(-90, imu);

        while (lDrive.isBusy()){ idle(); }

        lift1.setTargetPosition(2000);
        lift1.setPower(.99);
        Thread.sleep(500);

        setBothEncoder(550);
        setBothPower(.3);
        Thread.sleep(1000);

        grabber.setPosition(0.2);
        Thread.sleep(700);
        grabber.setPosition(0.5);
        Thread.sleep(1200);

        setBothEncoder(-400);
        setBothPower(.3);

        lift1.setTargetPosition(0);
        lift1.setPower(.5);
        while (lDrive.isBusy() || lift1.isBusy()){ idle(); }


        if (col != 1) {  //End turn

            setBothEncoder(-200);
            setBothPower(.3);
            while (lDrive.isBusy()) { idle(); }

            grabber.setPosition(0.8);
            Thread.sleep(700);
            grabber.setPosition(0.5);
            turn2Angle(-30, imu);
            while (lDrive.isBusy() || lift1.isBusy()) { idle(); }
            setBothEncoder(200);
            setBothPower(.3);
            while (lDrive.isBusy() || lift1.isBusy()) { idle(); }
        }

    }


    // Turning method for IMU
    void turn2Angle(double target, IMU_class i){
        lDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        initialHeading = getHeading(i);
        double relTarget = initialHeading + target;
        if (0 > target){
            //Turn right
            lDrive.setPower(.35);
            rDrive.setPower(-.35);
            while (Math.abs(relTarget - getHeading(i)) > 5){
                telemetry.addData("degrees to target", Math.abs(getHeading(i) - relTarget));
                telemetry.addData("current heading", getHeading(i));
                telemetry.update();
                idle();
            }
        }
        else{
            //Turn left
            lDrive.setPower(-.35);
            rDrive.setPower(.35);
            while (Math.abs(relTarget - getHeading(i)) > 5){
                telemetry.addData("degrees to target", Math.abs(getHeading(i) - relTarget));
                telemetry.addData("current heading", getHeading(i));
                telemetry.update();
                idle();
            }
        }
        setBothPower(0.0);
    }


    // Get heading for IMU
    double getHeading(IMU_class a){
        return a.getAngles()[0];
    }

    void setBothPower(double power){
        lDrive.setPower(power);
        rDrive.setPower(power);
    }
    void setBothEncoder(int encValue){
        lDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lDrive.setTargetPosition(encValue);
        rDrive.setTargetPosition(encValue);
    }
}