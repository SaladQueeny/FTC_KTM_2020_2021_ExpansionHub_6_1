/*
This program was written by the FTC KTM #12529 team at the Polytechnic University in 2020. 
  
   @author Kolpakov Egor
*/
package org.firstinspires.ftc.teamcode;

import android.os.CountDownTimer;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;



/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a four wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "KTM TeleOp 2020", group = "Linear Opmode")

//@Disabled
public class TeleOP2020 extends LinearOpMode {
    private static final int LED_CHANNEL = 5;
    //    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //Cha
    private DcMotor m6Intake = null;
    private DcMotor m5Lift = null;
    private Servo s5Shovel = null;

    //-------
    double magic(double input) {
        if(Math.abs(input)<0.02){
            double nol=0;
            return nol;
        } else{
            return Math.signum(input) * (0.9*Math.pow(Math.abs(input), 2)+0.1);
        }
    }
    //переменные для ПИД
    //int setpoint = 0;   // заданная величина, которую должен поддерживать регулятор
    //double input = 0;      // сигнал с датчика (например температура, которую мы регулируем)
    //double output = 0;     // выход с регулятора на управляющее устройство (например величина ШИМ или угол поворота серво)
    double pidMin = 0;     // минимальный выход с регулятора
    double pidMax = 255;   // максимальный выход с регулятора
    // коэффициенты
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 1.2;
    double _dt_s = 0.1; // время итерации в секундах//скорее всего получится выцепить из таймера
    // вспомогательные переменные
    double prevInput = 0;
    double integral = 0.0;

    //------------------
    double computePID(double input,double prevInput, double setpoint, double time, double time1) {
        double error = setpoint - input;           // ошибка регулирования
        double delta_input = prevInput - input;    // изменение входного сигнала
        prevInput = input;
        double output = 0;
        output += error * Kp;                  // пропорционально ошибке регулирования
        double dt=time1-time;
        output += delta_input * Kd / _dt_s;    // дифференциальная составляющая
        integral += error * Ki * _dt_s;        // расчёт интегральной составляющей
        // тут можно ограничить интегральную составляющую!
        output += integral;                           // прибавляем интегральную составляющую
        //output = constrain(output, pidMin, pidMax);   // ограничиваем выход
        return output;
    }

    /*
     * Functions declaration
     */
    //Lift claw
    void liftClaw(double lift_power) {
        m5Lift.setPower(lift_power);
    }

    void shovelTrigger(double shovel_pos) {
        s5Shovel.setPosition(shovel_pos);
    }

    void setmotorsPower(DcMotor motor1,DcMotor motor2, double power) {
        motor1.setPower(power);
        motor2.setPower(-power);

    }

    void setPowerTimed(DcMotor motor, double power, long milliseconds) {
        motor.setPower(power);
        sleep(milliseconds);
        motor.setPower(0);

    }

    void setPowerTimed(CRServo Crservo, double power, long milliseconds) {
        Crservo.setPower(power);
        sleep(milliseconds);
        Crservo.setPower(0);

    }
    protected double BatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation,double allowableDistanceError, DcMotor right_back, DcMotor left_front, DcMotor left_back, DcMotor right_front){
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget,distanceToYTarget);
        double time1=getRuntime();
        double time2 = getRuntime();
        while(!isStopRequested()&&distance>allowableDistanceError&& Math.abs(time1-time2)<5){
            time2 = getRuntime();
            distance = Math.hypot(distanceToXTarget,distanceToYTarget);
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
            double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
            double d1 = -pivotCorrection/40+robot_movement_y_component+robot_movement_x_component;
            double d2 = -pivotCorrection/40-robot_movement_y_component-robot_movement_x_component;
            double d3 = -pivotCorrection/40-robot_movement_y_component+robot_movement_x_component;
            double d4 = -pivotCorrection/40+robot_movement_y_component-robot_movement_x_component;
            double koeff = 0.5;
            setMotorsPowerOdom(d1,d2,d3,d4, right_back, left_front, left_back, right_front);
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
        stopMovement(right_back, left_front, left_back, right_front);
    }
    protected void setMotorsPowerOdom(double D1_power, double D2_power, double D3_power, double D4_power, DcMotor right_back, DcMotor left_front, DcMotor left_back, DcMotor right_front) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
        // Send power to wheels
        right_back.setPower(D1_power);
        left_front.setPower(D2_power);
        left_back.setPower(D3_power);
        right_front.setPower(D4_power);
    }
    protected void stopMovement(DcMotor right_back, DcMotor left_front, DcMotor left_back, DcMotor right_front){
        right_back.setPower(0);
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
    }
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

    /**
     * End of functions declaration
     */
    OdometryGlobalCoordinatePosition globalPositionUpdate;
    final double COUNTS_PER_INCH = 307.699557;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Chassis
        DcMotor m1Drive = hardwareMap.get(DcMotor.class, "m1 drive");
        DcMotor m2Drive = hardwareMap.get(DcMotor.class, "m2 drive");
        DcMotor m3Drive = hardwareMap.get(DcMotor.class, "m3 drive");
        DcMotor m4Drive = hardwareMap.get(DcMotor.class, "m4 drive");
        DcMotor m5Lift = hardwareMap.get(DcMotor.class, "m5 lift");
        DcMotor m6Intake = hardwareMap.get(DcMotor.class, "m6 intake");
        DcMotor m7ruletka = hardwareMap.get(DcMotor.class, "m7 rul");
        Servo s1RelicExtRet = hardwareMap.get(Servo.class, "s1 top claw");
        //s2_bottom_Claw = hardwareMap.get(CRServo.class, "s2 bottom claw");
        Servo s3Rotation = hardwareMap.get(Servo.class, "s3 rotation");
        Servo s4Kicker = hardwareMap.get(Servo.class, "s4 kick");
        Servo s5Shovel = hardwareMap.get(Servo.class, "s5 shovel");
        Servo s6RelicClaw = hardwareMap.get(Servo.class, "s6 relic claw");
        Servo s7RelicArm = hardwareMap.get(Servo.class, "s7 relic arm");
        DistanceSensor DistanceSensor_left = hardwareMap.get(DistanceSensor.class,"dist left");
        DistanceSensor DistanceSensor_right = hardwareMap.get(DistanceSensor.class,"dist right");
        //DistanceSensor DistanceSensor_back = hardwareMap.get(DistanceSensor.class,"dist back");
        //DistanceSensor DistanceSensor_forward = hardwareMap.get(DistanceSensor.class,"dist forward");
        BNO055IMU imu;
        Orientation angles;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //sensor
        //TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "sensor touch");
        //-------
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //m6Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //m6Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m1Drive.setDirection(DcMotor.Direction.REVERSE);
        m2Drive.setDirection(DcMotor.Direction.REVERSE);
        m3Drive.setDirection(DcMotor.Direction.REVERSE);
        m4Drive.setDirection(DcMotor.Direction.REVERSE);
        m5Lift.setDirection(DcMotor.Direction.FORWARD);
        m7ruletka.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m6Intake.setDirection(DcMotor.Direction.REVERSE);
        m7ruletka.setDirection(DcMotorSimple.Direction.FORWARD);
        //DcMotor m6Intake=hardwareMap.dcMotor.get("m6Intake");
        m1Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m5Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m6Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m7ruletka.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //m1Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runtime.reset();

        double m1DrivePower;
        double m2DrivePower;
        double m3DrivePower;
        double m4DrivePower;
        double m5LiftPower;
        double m6IntakePower=0;
        double m1m1m1=0;
        double m2m2m2=0;
        double m3m3m3=0;
        double m4m4m4=0;
        double m1m1m1m1=0;
        double m2m2m2m2=0;
        double m3m3m3m3=0;
        double m4m4m4m4=0;
        double m1m1=0;
        double m2m2=0;
        double m3m3=0;
        double m4m4=0;
        double m1left=0;
        double m1right=0;
        double m2left=0;
        double m2right=0;
        double m3left=0;
        double m3right=0;
        double m4left=0;
        double m4right=0;
        double m1DrivePowerforrotation=0;
        double m2DrivePowerforrotation=0;
        double m3DrivePowerforrotation=0;
        double m4DrivePowerforrotation=0;
        double m1DrivePowerfordrivetofoundation=0;
        double m2DrivePowerfordrivetofoundation=0;
        double m3DrivePowerfordrivetofoundation=0;
        double m4DrivePowerfordrivetofoundation=0;
        double m1DrivePowerfordrivetofoundation2=0;
        double m2DrivePowerfordrivetofoundation2=0;
        double m3DrivePowerfordrivetofoundation2=0;
        double m4DrivePowerfordrivetofoundation2=0;
        double m1DrivePowerfordrivetofoundation1=0;
        double m2DrivePowerfordrivetofoundation1=0;
        double m3DrivePowerfordrivetofoundation1=0;
        double m4DrivePowerfordrivetofoundation1=0;
        double m1DrivePowerfordrivetofoundation11=0;
        double m2DrivePowerfordrivetofoundation11=0;
        double m3DrivePowerfordrivetofoundation11=0;
        double m4DrivePowerfordrivetofoundation11=0;
        double time1;
        double time2;
        int nnnn=1;
        double a=0;
        double b=0;
        double prevangel=0;
        boolean shoot=false;
        String positionServo="not ready";
        //m1Drive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        s5Shovel.setPosition(0.2);
        double voltage = BatteryVoltage();
        double koeff = 13.0 / voltage;
        koeff = Math.pow(koeff, 1.25);
//        globalPositionUpdate = new OdometryGlobalCoordinatePosition(m1Drive, m2Drive, m4Drive, COUNTS_PER_INCH, 75);
//        Thread positionThread = new Thread(globalPositionUpdate);
//        positionThread.start();
//        globalPositionUpdate.reverseLeftEncoder();
        while (opModeIsActive()) {

//int ANDYMARK_TICKS_PER_REV = 1120;
            /*
             * Chassis movement
             */
            //Setup a variable for each drive wheel to save power level for telemetry


            // POV Mode uses right stick to go forward and right to slide.
            // - This uses basic math to combine motions and is easier to drive straight.
            double driveL = -gamepad1.left_stick_y;
            double driveR = -gamepad1.right_stick_y;
            float relic = gamepad2.left_stick_x;
            boolean ruletka_tuda = gamepad2.dpad_up;
            boolean ruletka_suda = gamepad2.dpad_down;
            double zagrebalo = -0.7*gamepad2.left_stick_y;
            double podiem = 1*gamepad2.right_stick_y;
            double slideR = 0.7*-gamepad1.left_trigger;
            double slideL = 0.7*gamepad1.right_trigger;
            double vpernazad=gamepad1.left_stick_y;
            double vleovpravo= -gamepad1.left_stick_x;
            double povorot= 0.87*gamepad1.right_stick_x;
            //DeviceInterfaceModule cdim = hardwareMap.deviceInterfaceModule.get("dim");
            //Slide Related
            slideL=magic(slideL);
            slideR=magic(slideR);
            povorot=magic(povorot);
            vpernazad=magic(vpernazad);
            //vleovpravo=magic(vleovpravo);
                /*m2DrivePower = povorot-vpernazad-vleovpravo;
                m4DrivePower = povorot+vpernazad-vleovpravo;
                m1DrivePower = povorot+vpernazad+vleovpravo;
                m3DrivePower = povorot-vpernazad+vleovpravo;*/
            m2DrivePower = (m2left+m2right+m1m1+m1m1m1+m1m1m1m1+m1DrivePowerfordrivetofoundation+m1DrivePowerfordrivetofoundation2+m1DrivePowerfordrivetofoundation1+m1DrivePowerfordrivetofoundation11) + povorot-vpernazad+(slideL+slideR)-(vleovpravo);
            m4DrivePower = (m4left+m4right+m2m2+m2m2m2+m2m2m2m2+m2DrivePowerfordrivetofoundation+m2DrivePowerfordrivetofoundation2+m2DrivePowerfordrivetofoundation1+m2DrivePowerfordrivetofoundation11)+ povorot+vpernazad+(slideL+slideR)-(vleovpravo);
            m1DrivePower = (m1left+m1right+m3m3+m3m3m3+m3m3m3m3+m3DrivePowerfordrivetofoundation+m3DrivePowerfordrivetofoundation2+m3DrivePowerfordrivetofoundation1+m3DrivePowerfordrivetofoundation11)+povorot+vpernazad+(slideL+slideR)+(vleovpravo);
            m3DrivePower = (m3left+m3right+m4m4+m4m4m4+m4m4m4m4+m4DrivePowerfordrivetofoundation+m4DrivePowerfordrivetofoundation2+m4DrivePowerfordrivetofoundation1+m4DrivePowerfordrivetofoundation11)+ povorot-vpernazad+(slideL+slideR)+(vleovpravo);
            double mochs=1;
            double max = Math.max(Math.max(m1DrivePower, m2DrivePower), Math.max(m3DrivePower, m4DrivePower));
            // Send calculated power to wheelsв
            if (max >= 1) {
                m1Drive.setPower(mochs*m1DrivePower *1/ max);
                m2Drive.setPower(mochs*m2DrivePower *1/ max);
                m3Drive.setPower(mochs*m3DrivePower *1/ max);
                m4Drive.setPower(mochs*m4DrivePower *1/ max);
            } else {
                m1Drive.setPower(mochs*m1DrivePower*1);
                m2Drive.setPower(mochs*m2DrivePower*1);
                m3Drive.setPower(mochs*m3DrivePower*1);
                m4Drive.setPower(mochs*m4DrivePower*1);
            }


            /*
             * End of chassis related code.
             */
            double servo5;

            if(podiem!=0&&a==0){
                if(podiem>0) {
                    m5Lift.setPower(podiem);
                }else{
                    m5Lift.setPower(podiem);
                }
            }else{m5Lift.setPower(0);}

            if(zagrebalo!=0){
                if(zagrebalo>0) {
                    m7ruletka.setPower(zagrebalo);
                }else{
                    m7ruletka.setPower(zagrebalo);
                }
            }else{m7ruletka.setPower(0);}


            if(gamepad2.right_bumper){
                s1RelicExtRet.setPosition(1);
            }
            if(gamepad2.left_bumper){
                s1RelicExtRet.setPosition(0);
            }
            //подъём магазина для катапульты
            if (gamepad2.y) { s4Kicker.setPosition(1); a=1;}
            if (gamepad2.a) { s4Kicker.setPosition(0); a=0;}
            //стрельба катапульты
            if(gamepad2.b){
                s5Shovel.setPosition(0);
                sleep(400);
                s5Shovel.setPosition(0.19);
                sleep(200);
            }


//----------------------------------------

            if(gamepad2.dpad_left){
                m6Intake.setPower(koeff*0.8);
            }else if(gamepad2.dpad_right||shoot){
                m6Intake.setPower(koeff*0.72);
            }else{
                m6Intake.setPower(0);
            }

//----------------------------------------
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if(gamepad1.y){
                if (angles.firstAngle > 5 || angles.firstAngle < -5) {
                    if (angles.firstAngle > 5) {
                        m1m1 = angles.firstAngle / 60;
                        m2m2 = angles.firstAngle / 60;
                        m3m3 = angles.firstAngle / 60;
                        m4m4 = angles.firstAngle / 60;
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    } else {
                        m1m1 = 0;
                        m2m2 = 0;
                        m3m3 = 0;
                        m4m4 = 0;
                    }
                    if (angles.firstAngle < -5) {
                        m1m1m1 = angles.firstAngle / 60;
                        m2m2m2 = angles.firstAngle / 60;
                        m3m3m3 = angles.firstAngle / 60;
                        m4m4m4 = angles.firstAngle / 60;
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    } else {
                        m1m1m1 = 0;
                        m2m2m2 = 0;
                        m3m3m3 = 0;
                        m4m4m4 = 0;
                    }
                }
            }else{
                m1m1 =0;
                m2m2=0;
                m3m3=0;
                m4m4=0;
                m1m1m1=0;
                m2m2m2=0;
                m3m3m3=0;
                m4m4m4=0;
                m1left = 0;
                m2left = 0;
                m3left = 0;
                m4left = 0;
                m1right=0;
                m2right=0;
                m3right=0;
                m4right=0;
            }
            if(gamepad1.left_bumper){
//                if (DistanceSensor_left.getDistance(DistanceUnit.CM) < 100) {
//                    m1left = -0.4;
//                    m2left = 0.4;
//                    m3left = -0.4;
//                    m4left = 0.4;
//                }
//                if (DistanceSensor_left.getDistance(DistanceUnit.CM) > 119) {
//                    m1right = 0.4;
//                    m2right = -0.4;
//                    m3right = 0.4;
//                    m4right = -0.4;
//                }
                m1Drive.setPower(0.4);
                m2Drive.setPower(-0.4);
                m3Drive.setPower(0.4);
                m4Drive.setPower(-0.4);

                sleep(1000);
            }
            if(gamepad1.left_bumper){
//                if (DistanceSensor_right.getDistance(DistanceUnit.CM) < 100) {
//                    m1right = 0.4;
//                    m2right = -0.4;
//                    m3right = 0.4;
//                    m4right = -0.4;
//                }
//                if (DistanceSensor_right.getDistance(DistanceUnit.CM) > 119) {
//                    m1left = -0.4;
//                    m2left = 0.4;
//                    m3left = -0.4;
//                    m4left = 0.4;
//                }
                m1Drive.setPower(-0.4);
                m2Drive.setPower(0.4);
                m3Drive.setPower(-0.4);
                m4Drive.setPower(0.4);

                sleep(1000);

            }
            if(gamepad1.x){
                angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if(angles.firstAngle>-5.5*nnnn){
                    m1Drive.setPower(0.2);
                    m2Drive.setPower(0.2);
                    m3Drive.setPower(0.2);
                    m4Drive.setPower(0.2);
                    angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                sleep(1000);
                nnnn+=1;

            }else{
                m1m1m1m1=0;
                m2m2m2m2=0;
                m3m3m3m3=0;
                m4m4m4m4=0;
            }
//----------------------------------------
            if(gamepad1.dpad_up){
                m1DrivePowerfordrivetofoundation11=0.3;
                m2DrivePowerfordrivetofoundation11=-0.3;//мотор енкодера с минусом
                m3DrivePowerfordrivetofoundation11=-0.3;//мотор енкодера с минусом
                m4DrivePowerfordrivetofoundation11=0.3;
            }else{
                m1DrivePowerfordrivetofoundation11=0;
                m2DrivePowerfordrivetofoundation11=0;
                m3DrivePowerfordrivetofoundation11=0;
                m4DrivePowerfordrivetofoundation11=0;
            }
            if(gamepad1.dpad_down){
                m1DrivePowerfordrivetofoundation1=-0.3;
                m2DrivePowerfordrivetofoundation1=0.3;
                m3DrivePowerfordrivetofoundation1=0.3;
                m4DrivePowerfordrivetofoundation1=-0.3;
            }else{
                m1DrivePowerfordrivetofoundation1=0;
                m2DrivePowerfordrivetofoundation1=0;
                m3DrivePowerfordrivetofoundation1=0;
                m4DrivePowerfordrivetofoundation1=0;
            }
            if(gamepad1.dpad_left){
                m1DrivePowerfordrivetofoundation=-0.33;
                m2DrivePowerfordrivetofoundation=-0.33;
                m3DrivePowerfordrivetofoundation=0.33;
                m4DrivePowerfordrivetofoundation=0.33;
            }else{
                m1DrivePowerfordrivetofoundation=0;
                m2DrivePowerfordrivetofoundation=0;
                m3DrivePowerfordrivetofoundation=0;
                m4DrivePowerfordrivetofoundation=0;
            }
            if(gamepad1.dpad_right){
                m1DrivePowerfordrivetofoundation2=0.33;
                m2DrivePowerfordrivetofoundation2=0.33;
                m3DrivePowerfordrivetofoundation2=-0.33;
                m4DrivePowerfordrivetofoundation2=-0.33;
            }else{
                m1DrivePowerfordrivetofoundation2=0;
                m2DrivePowerfordrivetofoundation2=0;
                m3DrivePowerfordrivetofoundation2=0;
                m4DrivePowerfordrivetofoundation2=0;
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("SErvo status", positionServo);
            telemetry.addData("SErvo position", s5Shovel.getPosition());
            telemetry.addData("Podiem position ", m7ruletka.getCurrentPosition());

            telemetry.addData("angleofrotate", angles.firstAngle);

            telemetry.addData("Distance left: ", DistanceSensor_left.getDistance(DistanceUnit.CM));
            telemetry.addData("Motors", "m1Drive (%.2f), m2Drive (%.1f), m3Drive (%.2f), m4Drive (%.2f)", m1DrivePower, m2DrivePower, m3DrivePower, m4DrivePower);
            telemetry.addData("Motors power for rotation", "m1Drive (%.2f), m2Drive (%.1f), m3Drive (%.2f), m4Drive (%.2f)", m1DrivePowerforrotation, m2DrivePowerforrotation, m3DrivePowerforrotation, m4DrivePowerforrotation);


            telemetry.addData("Vertical left encoder position", m1Drive.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", m2Drive.getCurrentPosition());
            telemetry.addData("horizontal encoder position", m4Drive.getCurrentPosition());
            telemetry.update();

        }
//        if(gamepad1.b){
//
//            shoot = true;
//
//            goToPosition(135* COUNTS_PER_INCH, -11*COUNTS_PER_INCH,0.8*koeff,0,6*COUNTS_PER_INCH, m1Drive,m2Drive,m3Drive,m4Drive);
//
//            goToPosition(175* COUNTS_PER_INCH, -11*COUNTS_PER_INCH,0.3*koeff,0,2*COUNTS_PER_INCH, m1Drive,m2Drive,m3Drive,m4Drive);
//        }
//        if(gamepad1.x){
//
//            shoot = true;
//
//            goToPosition(-165* COUNTS_PER_INCH, -11*COUNTS_PER_INCH,0.8*koeff,0,5*COUNTS_PER_INCH, m1Drive,m2Drive,m3Drive,m4Drive);
//
//            goToPosition(-220* COUNTS_PER_INCH, -11*COUNTS_PER_INCH,0.3*koeff,0,2*COUNTS_PER_INCH, m1Drive,m2Drive,m3Drive,m4Drive);
//        }
//        if(gamepad1.y){
//            a+=16;
//            s5Shovel.setPosition(0);
//            sleep(400);
//            s5Shovel.setPosition(0.19);
//            goToPosition((globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH+20)* COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate(),0.3*koeff,0,2*COUNTS_PER_INCH, m1Drive,m2Drive,m3Drive,m4Drive);
//
//        }
//        if(gamepad1.right_bumper){
//            shoot=false;
//        }

        //globalPositionUpdate.stop();
    }

}
