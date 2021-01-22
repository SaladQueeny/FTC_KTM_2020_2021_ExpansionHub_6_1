/*
This program was written by the FTC KTM #12529 team at the Polytechnic University in 2020.

This autonomy is the main one. She processes the number of rings and travels along a given trajectory

Our team wishes you all the best for the upcoming tournament.
All versions of the code starting from 2020 you can see here: https://github.com/RT-four/FTC-KTM-12529-TeamCode

Directed by RT-4(Philipp Vasiliev) and Dafter(Daniil Simonovsky (VK: https://vk.com/dafter_play))
*/

package org.firstinspires.ftc.teamcode.AutoOPs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVisionL;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "Right_Center", group = "AutoOP")
public class Right_Center extends Robot {
    private ElapsedTime runtime = new ElapsedTime();
    OpenCvInternalCamera phoneCam;
    EasyOpenCVVisionL pipeline;
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 307.699557;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "m4 drive", rbName = "m1 drive", lfName = "m2 drive", lbName = "m3 drive";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;
    @Override
    public void runOpMode() {
        initHW(hardwareMap);
        BNO055IMU imu;
        Orientation angles;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        // Camera setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new EasyOpenCVVisionL();
        phoneCam.setPipeline(pipeline);
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
        telemetry.addLine("Phone camera initialized");
        telemetry.update();
        s1TopClaw.setPosition(0);
        s4Kicker.setPosition(1);
        s3Rotation.setPosition(0);
        //Voltage regulation depending on the battery charge level
        double voltage = BatteryVoltage();
        double koeff = 13.0 / voltage;
        koeff = Math.pow(koeff, 1.25);
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
        globalPositionUpdate.reverseLeftEncoder();
        waitForStart();
        {

            imu.initialize(parameters);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.clear();
            telemetry.addData("Number of rings ", pipeline.position);
            telemetry.update();
            int countOfRings = 12;
            if ((pipeline.position == EasyOpenCVVisionL.RingPosition.FOUR)) {
                countOfRings = 4;
            }
            if ((pipeline.position == EasyOpenCVVisionL.RingPosition.ONE)) {
                countOfRings = 1;
            }
            if ((pipeline.position == EasyOpenCVVisionL.RingPosition.NONE)) {
                countOfRings = 0;
            }
            //Voltage regulation depending on the battery charge level
            telemetry.addData("count_of_rings ", countOfRings);
            telemetry.update();
            boolean check= true;

            //shooting targets


            Shooting(koeff,0.68);
            goToPosition(2* COUNTS_PER_INCH, -170*COUNTS_PER_INCH,0.5*koeff,0,7*COUNTS_PER_INCH);
            goToPosition(2* COUNTS_PER_INCH, -227*COUNTS_PER_INCH,0.3*koeff,0,2*COUNTS_PER_INCH);
            Shoot();

            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            while(!isStopRequested()&&angles.firstAngle<5.5){
                setMotorsPower(-0.15*koeff,-0.15*koeff,-0.15*koeff,-0.15*koeff);
                angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            setMotorsPower(0,0,0,0);
            Shooting(koeff,0.7);
            Shoot();

            while(!isStopRequested()&&angles.firstAngle<11){
                setMotorsPower(-0.15*koeff,-0.15*koeff,-0.15*koeff,-0.15*koeff);
                angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            setMotorsPower(0,0,0,0);
            Shoot();

            endShooting();
            s3Rotation.setPosition(1);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            while (!isStopRequested() && angles.firstAngle > 5) {
                setMotorsPower(0.3 * koeff, 0.3 * koeff, 0.3 * koeff, 0.3 * koeff);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            setMotorsPower(0, 0, 0, 0);
            s3Rotation.setPosition(1);
            s4Kicker.setPosition(0);

/*

            //shooting high gates

            Shooting(koeff,0.75);
            goToPosition(-12* COUNTS_PER_INCH, -150*COUNTS_PER_INCH,0.5*koeff,0,7*COUNTS_PER_INCH);
            goToPosition(-12* COUNTS_PER_INCH, -207*COUNTS_PER_INCH,0.3*koeff,0,2*COUNTS_PER_INCH);
            sleep(300);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            while (!isStopRequested() && angles.firstAngle < 6) {
                setMotorsPower(-0.18 * koeff, -0.18 * koeff, -0.18 * koeff, -0.18 * koeff);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            setMotorsPower(0, 0, 0, 0);
            sleep(500);
            Shooting(koeff, 0.78);
            Shoot();
            sleep(100);
            Shoot();
            sleep(100);
            Shoot();
            s4Kicker.setPosition(0);
            endShooting();
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            while (!isStopRequested() && angles.firstAngle > 2) {
                setMotorsPower(0.3 * koeff, 0.3 * koeff, 0.3 * koeff, 0.3 * koeff);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            setMotorsPower(0, 0, 0, 0);
            s3Rotation.setPosition(1);

*/

            //moving back to rings
            if(countOfRings==4&&check) {
                check=false;
                goToPosition(-5 * COUNTS_PER_INCH, -125 * COUNTS_PER_INCH, 0.6 * koeff, 0, 8 * COUNTS_PER_INCH);

                goToPosition(-74 * COUNTS_PER_INCH, -105 * COUNTS_PER_INCH, 0.2 * koeff, 0, 2 * COUNTS_PER_INCH);

                //collecting rings
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double time1=getRuntime();
                double time2 = getRuntime();
                while(Math.abs(time1-time2)<0.2){
                    setMotorsPowerforvard(-0.7,0.7,0.7,-0.7, angles, imu, 0);
                    time2=getRuntime();
                }

                //setMotorsPower(-0.7,0.7,0.7,-0.7);
                //sleep(200);
                m5Lift.setPower(-1);
                setMotorsPower(0,0,0,0);
                sleep(900);

                setMotorsPower(-0.2,0.2,0.2,-0.2);
                sleep(550);
                setMotorsPower(0,0,0,0);
                sleep(900);

                setMotorsPower(-0.2,0.2,0.2,-0.2);
                sleep(550);
                setMotorsPower(0,0,0,0);
                sleep(1600);

                m5Lift.setPower(0);
                s4Kicker.setPosition(1);
                sleep(500);



                //shooting high gates
                Shooting(koeff, 0.77);
                sleep(500);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                while (!isStopRequested() && angles.firstAngle > -4) {
                    setMotorsPower(0.1 * koeff, 0.1 * koeff, 0.1 * koeff, 0.1 * koeff);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                setMotorsPower(0, 0, 0, 0);
                sleep(500);
                Shooting(koeff, 0.78);
                Shoot();
                sleep(100);
                Shoot();
                sleep(100);
                Shoot();
                s4Kicker.setPosition(0);
                endShooting();
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                while (!isStopRequested() && angles.firstAngle < -2) {
                    setMotorsPower(-0.3 * koeff, -0.3 * koeff, -0.3 * koeff, -0.3 * koeff);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                setMotorsPower(0, 0, 0, 0);


                goToPosition(-40* COUNTS_PER_INCH,-380*COUNTS_PER_INCH,0.6*koeff,0,10*COUNTS_PER_INCH);

                //m5Lift.setPower(-1);
                goToPosition(-140* COUNTS_PER_INCH,-455*COUNTS_PER_INCH,0.3*koeff,0,3*COUNTS_PER_INCH);
                s3Rotation.setPosition(0);
                otpustivobl(koeff);


                setMotorsPower(0,0,0,0);
                goToPosition(-45 * COUNTS_PER_INCH, -430 * COUNTS_PER_INCH, 0.5*koeff, 0, 8* COUNTS_PER_INCH);

                goToPosition(-5* COUNTS_PER_INCH, -300 * COUNTS_PER_INCH, 0.4*koeff, 0, 4* COUNTS_PER_INCH);

            }
            if(countOfRings==1&&check){//добавить координату
                check=false;
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                while (!isStopRequested() && angles.firstAngle < -5) {
                    setMotorsPower(-0.3 * koeff, -0.3 * koeff, -0.3 * koeff, -0.3 * koeff);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                setMotorsPower(0, 0, 0, 0);
                s4Kicker.setPosition(0);
                m5Lift.setPower(-1);

                goToPosition(-5 * COUNTS_PER_INCH, -130 * COUNTS_PER_INCH, 0.5 * koeff, 0, 8 * COUNTS_PER_INCH);
                goToPosition(-75 * COUNTS_PER_INCH, -110 * COUNTS_PER_INCH, 0.3 * koeff, 0, 2 * COUNTS_PER_INCH);

                goToPosition(-75 * COUNTS_PER_INCH, -150 * COUNTS_PER_INCH, 0.15 * koeff, 0, 2 * COUNTS_PER_INCH);
                sleep(1000);
                s4Kicker.setPosition(1);
                sleep(500);
                Shooting(koeff, 0.78);

                sleep(500);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                while (!isStopRequested() && angles.firstAngle > -4) {
                    setMotorsPower(0.18 * koeff, 0.18 * koeff, 0.18 * koeff, 0.18 * koeff);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                setMotorsPower(0, 0, 0, 0);
                m5Lift.setPower(0);
                sleep(500);
                Shoot();
                sleep(100);

                Shoot();
                endShooting();
                s4Kicker.setPosition(0);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                while (!isStopRequested() && angles.firstAngle < -5) {
                    setMotorsPower(-0.3 * koeff, -0.3 * koeff, -0.3 * koeff, -0.3 * koeff);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                setMotorsPower(0, 0, 0, 0);
                //добавить координату

                //m5Lift.setPower(-1);
                goToPosition(-50* COUNTS_PER_INCH,-350*COUNTS_PER_INCH,0.3*koeff,0,2*COUNTS_PER_INCH);
                s3Rotation.setPosition(0);
                otpustivobl(koeff);

                setMotorsPower(0,0,0,0);

                goToPosition(-5 * COUNTS_PER_INCH, -300 * COUNTS_PER_INCH, 0.3*koeff, 0, 3 * COUNTS_PER_INCH);

            }
            if(countOfRings==0&&check){//добавить координату
                check=false;
                goToPosition(-80* COUNTS_PER_INCH,-275*COUNTS_PER_INCH,0.5*koeff,0,7*COUNTS_PER_INCH);

                //m5Lift.setPower(-1);
                goToPosition(-134* COUNTS_PER_INCH,-275*COUNTS_PER_INCH,0.3*koeff,0,2*COUNTS_PER_INCH);
                s3Rotation.setPosition(0);
                otpustivobl(koeff);


                angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                while(!isStopRequested()&&angles.firstAngle<-5){
                    setMotorsPower(-0.3*koeff,-0.3*koeff,-0.3*koeff,-0.3*koeff);
                    angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                setMotorsPower(0,0,0,0);

                goToPosition(-5 * COUNTS_PER_INCH, -300 * COUNTS_PER_INCH, 0.3*koeff, 0, 3 * COUNTS_PER_INCH);

            }
            //добавить координату

            globalPositionUpdate.stop();

//            //otpustivobl(koeff);
//            Shooting(koeff,0.82);
//            double time1;
//            double time2;
//            time1=getRuntime();
//            time2=getRuntime();
//            while(!isStopRequested()&&DistanceSensor_right.getDistance(DistanceUnit.CM)>32){
//                setMotorsPowerright(-0.3*koeff,0.3*koeff,-0.3*koeff,0.3*koeff,angles, imu, 0);
//                time2=getRuntime();
//            }
//            setMotorsPower(0,0,0,0);
//            time1=getRuntime();
//            time2=getRuntime();
//            while(!isStopRequested()&&time2-time1<0.2){
//                //angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                setMotorsPowerforvard(0.5*koeff,-0.5*koeff,-0.5*koeff,0.5*koeff,angles, imu, 0);
//                time2=getRuntime();
//            }
//            setMotorsPower(0,0,0,0);
//            sleep(1000);
//            Shoot();
//            sleep(1000);
//            Shoot();
//            sleep(1000);
//            Shoot();
//            endShooting();
//            while(!isStopRequested()&&DistanceSensor_right.getDistance(DistanceUnit.CM)>20){
//                setMotorsPowerright(-0.3*koeff,0.3*koeff,-0.3*koeff,0.3*koeff,angles, imu, 0);
//                time2=getRuntime();
//            }
//            setMotorsPower(0,0,0,0);
//
//            // The choice of the direction of movement depending on the number of rings
//            boolean check =true;
//            if (countOfRings==4&&check) {
//                check=false;
//                time1=getRuntime();
//                time2=getRuntime();
//                while(!isStopRequested()&&DistanceSensor_forward.getDistance(DistanceUnit.CM)>69){
//                    setMotorsPowerforvard(-0.5*koeff,0.5*koeff,0.5*koeff,-0.5*koeff,angles, imu, 0);
//                    time2=getRuntime();
////                    telemetry.addData("angle of rotate ", angles.firstAngle);
////                    telemetry.update();
//                }
//                setMotorsPower(0,0,0,0);
//
//                otpustivobl(koeff);
//                time1=getRuntime();
//                time2=getRuntime();
//                while(!isStopRequested()&&time2-time1<0.2){
//                    setMotorsPowerright(-0.3*koeff,0.3*koeff,-0.3*koeff,0.3*koeff,angles, imu, 0);
//                    time2=getRuntime();
//                }
//                setMotorsPower(0,0,0,0);
//                time1=getRuntime();
//                time2=getRuntime();
//                while(!isStopRequested()&&DistanceSensor_forward.getDistance(DistanceUnit.CM)<160){
//                    setMotorsPowerback(0.5*koeff,-0.5*koeff,-0.5*koeff,0.5*koeff,angles, imu, 0);
//                    time2=getRuntime();
//                }
//                setMotorsPower(0,0,0,0);
//
//            }
//
//            if (countOfRings==1&&check) {
//                check=false;
//                time1=getRuntime();
//                time2=getRuntime();
//                while(!isStopRequested()&&DistanceSensor_forward.getDistance(DistanceUnit.CM)>80){
//                    setMotorsPowerforvard(-0.5*koeff,0.5*koeff,0.5*koeff,-0.5*koeff,angles, imu, 0);
//                    time2=getRuntime();
//                }
//                setMotorsPower(0,0,0,0);
//
//                angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                while(!isStopRequested()&&angles.firstAngle<82){
//                    setMotorsPower(-0.3*koeff,-0.3*koeff,-0.3*koeff,-0.3*koeff);
//                    angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                }
//                setMotorsPower(0,0,0,0);
//                time1=getRuntime();
//                time2=getRuntime();
//                while(!isStopRequested()&&time2-time1<0.2){
//                    setMotorsPowerforvard(-0.5*koeff,0.5*koeff,0.5*koeff,-0.5*koeff,angles, imu, -90);
//                    time2=getRuntime();
//                }
//                setMotorsPower(0,0,0,0);
//                otpustivobl(koeff);
//
//                angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                while(!isStopRequested()&&angles.firstAngle>5){
//                    setMotorsPower(0.3*koeff,0.3*koeff,0.3*koeff,0.3*koeff);
//                    angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                }
//                setMotorsPower(0,0,0,0);
//
//                time1=getRuntime();
//                time2=getRuntime();
//                while(!isStopRequested()&&DistanceSensor_forward.getDistance(DistanceUnit.CM)<160){
//                    angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                    setMotorsPowerback(0.5*koeff,-0.5*koeff,-0.5*koeff,0.5*koeff,angles, imu, 0);
//                    time2=getRuntime();
//                }
//                setMotorsPower(0,0,0,0);
//
//            }
//
//            if (countOfRings==0&&check) {
//                check=false;
//                time1=getRuntime();
//                time2=getRuntime();
//                while(!isStopRequested()&&DistanceSensor_forward.getDistance(DistanceUnit.CM)>160){
//                    setMotorsPowerforvard(-0.5*koeff,0.5*koeff,0.5*koeff,-0.5*koeff,angles, imu, 0);
//                    time2=getRuntime();
//                }
//                setMotorsPower(0,0,0,0);
//                time1=getRuntime();
//                time2=getRuntime();
//                while(!isStopRequested()&&time2-time1<0.8){
//                    //angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                    setMotorsPowerforvard(0.5*koeff,-0.5*koeff,-0.5*koeff,0.5*koeff,angles, imu, 0);
//                    time2=getRuntime();
//                }
//                setMotorsPower(0,0,0,0);
//                sleep(200);
//                otpustivobl(koeff);
//                time1=getRuntime();
//                time2=getRuntime();
//                while(!isStopRequested()&&time2-time1<0.2){
//                    //angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                    setMotorsPowerforvard(0.5*koeff,-0.5*koeff,-0.5*koeff,0.5*koeff,angles, imu, 0);
//                    time2=getRuntime();
//                }
//                setMotorsPower(0,0,0,0);


            //}
        }


    }
    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation,double allowableDistanceError){
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget,distanceToYTarget);

        while(!isStopRequested()&&distance>allowableDistanceError){

            distance = Math.hypot(distanceToXTarget,distanceToYTarget);
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
            double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
            double d1 = -pivotCorrection/60+robot_movement_y_component+robot_movement_x_component;
            double d2 = -pivotCorrection/60-robot_movement_y_component-robot_movement_x_component;
            double d3 = -pivotCorrection/60-robot_movement_y_component+robot_movement_x_component;
            double d4 = -pivotCorrection/60+robot_movement_y_component-robot_movement_x_component;
            double koeff = 0.5;
            setMotorsPowerOdom(d1,d2,d3,d4);
//            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
//            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
//            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
//            telemetry.addData("robot_movement_x_component", robot_movement_x_component);
//            telemetry.addData("robot_movement_y_component", robot_movement_y_component);
//            telemetry.addData("pivot", pivotCorrection);
//            telemetry.addData("motor1_power", d1);
//            telemetry.addData("motor2_power", d2);
//            telemetry.addData("motor3_power", d3);
//            telemetry.addData("motor4_power", d4);
//            telemetry.update();
        }
        stopMovement();
    }
    protected void setMotorsPowerOdom(double D1_power, double D2_power, double D3_power, double D4_power) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
        // Send power to wheels
        right_back.setPower(D1_power);
        left_front.setPower(D2_power);
        left_back.setPower(D3_power);
        right_front.setPower(D4_power);
    }
    protected void stopMovement(){
        right_back.setPower(0);
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        //right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        //right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}