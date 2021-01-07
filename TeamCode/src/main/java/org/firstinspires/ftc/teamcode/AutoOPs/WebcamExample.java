///*
// * Copyright (c) 2019 OpenFTC Team
// *
// * Permission is hereby granted, free of charge, to any person obtaining a copy
// * of this software and associated documentation files (the "Software"), to deal
// * in the Software without restriction, including without limitation the rights
// * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// * copies of the Software, and to permit persons to whom the Software is
// * furnished to do so, subject to the following conditions:
// *
// * The above copyright notice and this permission notice shall be included in all
// * copies or substantial portions of the Software.
// * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// * SOFTWARE.
// */
//
//package org.firstinspires.ftc.teamcode.AutoOPs;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVisionL;
//import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//
//@Autonomous(name = "Webcamtest_left", group = "AutoOP")
//public class WebcamExample extends Robot
//{
//    private ElapsedTime runtime = new ElapsedTime();
//    OpenCvCamera webcam;
//    org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVisionL pipeline = new org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVisionL();
//
//
//    DcMotor right_front, right_back, left_front, left_back;
//    //Odometry Wheels
//    DcMotor verticalLeft, verticalRight, horizontal;
//
//    final double COUNTS_PER_INCH = 307.699557;
//
//    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
//    String rfName = "m4 drive", rbName = "m1 drive", lfName = "m2 drive", lbName = "m3 drive";
//    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;
//
//    OdometryGlobalCoordinatePosition globalPositionUpdate;
//    @Override
//    public void runOpMode()
//    {
//        initHW(hardwareMap);
//        BNO055IMU imu;
//        Orientation angles;
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        webcam.setPipeline(pipeline);
//        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//        });
//
//        telemetry.addLine("Phone camera initialized");
//        telemetry.update();
//        s1TopClaw.setPosition(0);
//        s4Kicker.setPosition(1);
//
//        double voltage = BatteryVoltage();
//        double koeff = 13.0 / voltage;
//        koeff = Math.pow(koeff, 1.25);
//        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
//        Thread positionThread = new Thread(globalPositionUpdate);
//        positionThread.start();
//        globalPositionUpdate.reverseLeftEncoder();
//
//        waitForStart();
//        {
//            imu.initialize(parameters);
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            telemetry.clear();
//            //telemetry.addData("Number of rings ", pipeline.position);
//            telemetry.update();
//            int countOfRings=12;
//            if((pipeline.position == org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVisionL.RingPosition.FOUR)){
//                countOfRings = 4;
//            }
//            if((pipeline.position == org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVisionL.RingPosition.ONE)){
//                countOfRings = 1;
//            }
//            if((pipeline.position == org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVisionL.RingPosition.NONE)){
//                countOfRings = 0;
//            }
//
//            telemetry.addData("count_of_rings ", countOfRings);
//            telemetry.update();
////            boolean check= true;
////
////            Shooting(koeff,0.8);
////            goToPosition(35* COUNTS_PER_INCH, -217*COUNTS_PER_INCH,0.3*koeff,0,2*COUNTS_PER_INCH);
////            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
////            while(!isStopRequested()&&angles.firstAngle>-16){
////                setMotorsPower(0.3*koeff,0.3*koeff,0.3*koeff,0.3*koeff);
////                angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
////            }
////            setMotorsPower(0,0,0,0);
////            Shoot();
////            //sleep(1000);
////            Shoot();
////            //sleep(1000);
////            Shoot();
////            endShooting();
////            //goToPosition(-40* COUNTS_PER_INCH, -217*COUNTS_PER_INCH,0.3,40,2*COUNTS_PER_INCH);
////            sleep(1000);
////
////
////            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
////            while(!isStopRequested()&&angles.firstAngle<-5){
////                setMotorsPower(-0.3*koeff,-0.3*koeff,-0.3*koeff,-0.3*koeff);
////                angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
////            }
////            setMotorsPower(0,0,0,0);
////            if(countOfRings==0&&check){
////                check=false;
////                goToPosition(28* COUNTS_PER_INCH,-274* COUNTS_PER_INCH,0.3*koeff,0,2*COUNTS_PER_INCH);
////                otpustivobl(koeff);
////                //goToPosition(28 * COUNTS_PER_INCH, -274 * COUNTS_PER_INCH, 0.3*koeff, 0, 2 * COUNTS_PER_INCH);
////
////            }
////            if(countOfRings==4&&check){
////                check=false;
////                goToPosition(28* COUNTS_PER_INCH,-455*COUNTS_PER_INCH,0.3*koeff,0,2*COUNTS_PER_INCH);
////                otpustivobl(koeff);
////                goToPosition(24 * COUNTS_PER_INCH, -284 * COUNTS_PER_INCH, 0.3*koeff, 0, 2 * COUNTS_PER_INCH);
////
////            }
////            if(countOfRings==1&&check){
////                check=false;
////                goToPosition(18* COUNTS_PER_INCH,-394*COUNTS_PER_INCH,0.3*koeff,0,2*COUNTS_PER_INCH);
////                angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
////                while(!isStopRequested()&&angles.firstAngle>-85){
////                    setMotorsPower(0.3*koeff,0.3*koeff,0.3*koeff,0.3*koeff);
////                    angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
////                }
////                setMotorsPower(0,0,0,0);
////                sleep(1000);
////                otpustivobl(koeff);
////
////
////                angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
////                while(!isStopRequested()&&angles.firstAngle<-5){
////                    setMotorsPower(-0.3*koeff,-0.3*koeff,-0.3*koeff,-0.3*koeff);
////                    angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
////                }
////                setMotorsPower(0,0,0,0);
////                goToPosition(24 * COUNTS_PER_INCH, -284 * COUNTS_PER_INCH, 0.3*koeff, 0, 2 * COUNTS_PER_INCH);
////
////            }
////            //goToPosition(24 * COUNTS_PER_INCH, -284 * COUNTS_PER_INCH, 0.3*koeff, 0, 2 * COUNTS_PER_INCH);
////
////
//
//            globalPositionUpdate.stop();
//        }
//
//    }
//
//    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation,double allowableDistanceError){
//        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
//        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
//
//        double distance = Math.hypot(distanceToXTarget,distanceToYTarget);
//
//        while(!isStopRequested()&&distance>allowableDistanceError){
//
//            distance = Math.hypot(distanceToXTarget,distanceToYTarget);
//            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
//            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
//
//            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
//
//            double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
//            double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
//            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
//            double d1 = -pivotCorrection/70+robot_movement_y_component+robot_movement_x_component;
//            double d2 = -pivotCorrection/70-robot_movement_y_component-robot_movement_x_component;
//            double d3 = -pivotCorrection/70-robot_movement_y_component+robot_movement_x_component;
//            double d4 = -pivotCorrection/70+robot_movement_y_component-robot_movement_x_component;
//            double koeff = 0.5;
//            setMotorsPowerOdom(d1,d2,d3,d4);
////            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
////            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
////            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
////            telemetry.addData("robot_movement_x_component", robot_movement_x_component);
////            telemetry.addData("robot_movement_y_component", robot_movement_y_component);
////            telemetry.addData("pivot", pivotCorrection);
////            telemetry.addData("motor1_power", d1);
////            telemetry.addData("motor2_power", d2);
////            telemetry.addData("motor3_power", d3);
////            telemetry.addData("motor4_power", d4);
////            telemetry.update();
//        }
//        stopMovement();
//    }
//    protected void setMotorsPowerOdom(double D1_power, double D2_power, double D3_power, double D4_power) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
//        // Send power to wheels
//        right_back.setPower(D1_power);
//        left_front.setPower(D2_power);
//        left_back.setPower(D3_power);
//        right_front.setPower(D4_power);
//    }
//    protected void stopMovement(){
//        right_back.setPower(0);
//        left_front.setPower(0);
//        left_back.setPower(0);
//        right_front.setPower(0);
//    }
//
//    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
//        right_front = hardwareMap.dcMotor.get(rfName);
//        right_back = hardwareMap.dcMotor.get(rbName);
//        left_front = hardwareMap.dcMotor.get(lfName);
//        left_back = hardwareMap.dcMotor.get(lbName);
//
//        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
//        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
//        horizontal = hardwareMap.dcMotor.get(hEncoderName);
//
//        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        //left_front.setDirection(DcMotorSimple.Direction.REVERSE);
//        //right_front.setDirection(DcMotorSimple.Direction.REVERSE);
//        //right_back.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        telemetry.addData("Status", "Hardware Map Init Complete");
//        telemetry.update();
//    }
//
//    /**
//     * Calculate the power in the x direction
//     * @param desiredAngle angle on the x axis
//     * @param speed robot's speed
//     * @return the x vector
//     */
//    private double calculateX(double desiredAngle, double speed) {
//        return Math.sin(Math.toRadians(desiredAngle)) * speed;
//    }
//
//    /**
//     * Calculate the power in the y direction
//     * @param desiredAngle angle on the y axis
//     * @param speed robot's speed
//     * @return the y vector
//     */
//    private double calculateY(double desiredAngle, double speed) {
//        return Math.cos(Math.toRadians(desiredAngle)) * speed;
//    }
//}
/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.AutoOPs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVisionL;
@TeleOp
public class WebcamExample extends LinearOpMode
{
    OpenCvCamera webcam;

    @Override
    public void runOpMode()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVisionL pipeline = new org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVisionL();
        webcam.setPipeline(pipeline);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("pipeline working", pipeline.position);
            telemetry.update();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}