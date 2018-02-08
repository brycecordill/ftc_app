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

@Autonomous(name = "Auto Blue PARALLEL Sensor", group = "3650 New")
public class Auto_Blue_Para_Sensor extends LinearOpMode{
    private VuforiaLocalizer vuforia;
    private PrivateData priv = new PrivateData();

    private DcMotor lDrive, rDrive, lift1;
    private Servo armServo, grabber;
    private ColorSensor cSensor, cSensor2;

    private double initialHeading;

    @Override
    public void runOpMode() throws InterruptedException {
        double armExtendedPos = 0.1;
        double armRetractedPos = 0.6;

        IMU_class imu = new IMU_class("imu", hardwareMap);
        initialHeading = getHeading(imu);

        rDrive = hardwareMap.dcMotor.get("rDrive");
        rDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lDrive = hardwareMap.dcMotor.get("lDrive");
        lDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rDrive.setDirection(DcMotor.Direction.REVERSE);

        lift1 = hardwareMap.dcMotor.get("lift1");
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setDirection(DcMotor.Direction.REVERSE);

        armServo = hardwareMap.servo.get("armServo");
        cSensor = hardwareMap.colorSensor.get("cSensor");
        cSensor2 = hardwareMap.colorSensor.get("cSensor2");

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
        Thread.sleep(900);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift1.setTargetPosition(1000);
        lift1.setPower(.7);

        //Down servo
        armServo.setPosition(armExtendedPos);

        relicTrackables.activate(); // Start Vuforia object search
        Thread.sleep(1000);

        int col = -2; //1=R, 0=C, -1=L, -2=undetected
        long startTime = System.currentTimeMillis();


        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < 3000) { //Run for 3 secs
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("Key: ", vuMark);
                telemetry.update();
                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    col = -1;
                    break;
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    col = 0;
                    break;
                } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                    col = 1;
                    break;
                }

            } else {
                telemetry.addData("No key detected!", null);
            }
            telemetry.update();
            idle();
        }


        relicTrackables.deactivate(); // End Vuforia search

        if (cSensor.blue() > cSensor.red()) {
            //Move forward if blue
            setBothEncoder(300);
            setBothPower(.3);
            while (opModeIsActive() && lDrive.isBusy()) {
                idle();
            }
            armServo.setPosition(armRetractedPos);
            setBothEncoder(-300);
            setBothPower(.4);
            while (opModeIsActive() && lDrive.isBusy()) {
                idle();
            }
        }

        // Move far backward
        setBothEncoder(-2700);
        setBothPower(0.4);
        Thread.sleep(300);

        while (opModeIsActive() && lDrive.isBusy()) {
            idle();
        }
        setBothPower(0.0);
        armServo.setPosition(armRetractedPos);

        //Run forwards into stone (gets a consistent staring pos)
        //setBothEncoder(400);
        //setBothPower(0.3);

        while (opModeIsActive() && lDrive.isBusy()) {
            idle();
        }
        Thread.sleep(1000);

        findCenter();


        if (col == 1) { //L
            setBothEncoder(2000); //Good
            Thread.sleep(200);
            setBothPower(.3);
        } else if (col == 0) { //C
            setBothEncoder(950); //Good
            Thread.sleep(200);
            setBothPower(.3);
        } else if (col == -1) { //R
            setBothEncoder(150); //Good
            Thread.sleep(200);
            setBothPower(.3);
        } else { //C
            setBothEncoder(950);
            Thread.sleep(200);
            setBothPower(.3);
        }

        while (opModeIsActive() && lDrive.isBusy()) {
            idle();
        }

        turn2Angle(-90, imu);

        while (opModeIsActive() && lDrive.isBusy()) {
            idle();
        }

        lift1.setTargetPosition(400);
        lift1.setPower(.7);
        Thread.sleep(500);

        setBothEncoder(600);
        setBothPower(.3);
        Thread.sleep(1000);

        grabber.setPosition(0.2);
        Thread.sleep(700);

        setBothEncoder(-800);
        setBothPower(.3);

        lift1.setTargetPosition(100);
        lift1.setPower(.5);
        while (opModeIsActive() && lDrive.isBusy() || lift1.isBusy()) {
            idle();
        }
    }

    //Turning method for IMU
    void turn2Angle(double target, IMU_class i){
        lDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        initialHeading = getHeading(i);
        double relTarget = initialHeading + target;
        if (0 > target){
            //Turn right
            lDrive.setPower(.35);
            rDrive.setPower(-.35);
            while (opModeIsActive() && Math.abs(relTarget - getHeading(i)) > 5){
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
            while (opModeIsActive() && Math.abs(relTarget - getHeading(i)) > 5){
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

    protected void setBothPower(double power){
        lDrive.setPower(power);
        rDrive.setPower(power);
    }
    private void setBothEncoder(int encValue){
        lDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lDrive.setTargetPosition(encValue);
        rDrive.setTargetPosition(encValue);
    }

    private void findCenter(){
        int blueThresh = 6, grayThresh = 3; //Check values
        boolean blueDetected1 = false, grayDetected1 = false, blueDetected2 =false, grayDetected2 = false;
        lDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Encoders: ", lDrive.getCurrentPosition());

        setBothPower(-.3);

        long startTime = System.currentTimeMillis();
        while (!blueDetected1 && opModeIsActive() && System.currentTimeMillis() - startTime < 6000){
            blueDetected1 = (cSensor2.blue() - (cSensor2.red() + cSensor2.green()) / 2 > blueThresh);
            //Check tail value
            if(blueDetected1) {
                telemetry.addLine("First blue");
                telemetry.update();
            }
        }

        int startEnc = lDrive.getCurrentPosition();

        startTime = System.currentTimeMillis();
        while (!grayDetected1 && opModeIsActive() && System.currentTimeMillis() - startTime < 2000){
            grayDetected1 = Math.abs(cSensor2.blue() - (cSensor2.red() + cSensor2.green()) / 2) < grayThresh;
            //Check tail value
            if(grayDetected1) {
                telemetry.addData("Encoders: ", lDrive.getCurrentPosition());
                telemetry.addLine("First gray");
                telemetry.update();
            }
        }
        startTime = System.currentTimeMillis();
        while (!blueDetected2 && opModeIsActive() && System.currentTimeMillis() - startTime < 3000){
            //if the red val - the avg of the blue & green is less than x ...
            blueDetected2 = (cSensor2.blue() - (cSensor2.red() + cSensor2.green()) / 2 > blueThresh);
            //Check tail value
            if(blueDetected2) {
                telemetry.addData("Encoders: ", lDrive.getCurrentPosition());
                telemetry.addLine("Second blue");
                telemetry.update();
            }
        }

        startTime = System.currentTimeMillis();
        while (!grayDetected2 && opModeIsActive() && System.currentTimeMillis() - startTime < 2000){
            grayDetected2 = Math.abs(cSensor2.blue() - (cSensor2.red() + cSensor2.green()) / 2) < grayThresh;
            //Check tail value
            if(grayDetected2) {
                telemetry.addData("Encoders: ", lDrive.getCurrentPosition());
                telemetry.addLine("Second gray");
                telemetry.update();
            }
        }

        setBothPower(0.0);
        int middlePos = (lDrive.getCurrentPosition() + startEnc) / 2;
        telemetry.addData("Start position", startEnc);
        telemetry.addData("Current position", lDrive.getCurrentPosition());
        telemetry.addData("encDistance", middlePos);
        telemetry.update();
        startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < 200){
            idle();
        }

        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lDrive.setTargetPosition(middlePos);
        rDrive.setTargetPosition(middlePos);
        setBothPower(.3);
        while (opModeIsActive() && lDrive.isBusy()){ idle(); }
        setBothPower(0.0);
    }
}
