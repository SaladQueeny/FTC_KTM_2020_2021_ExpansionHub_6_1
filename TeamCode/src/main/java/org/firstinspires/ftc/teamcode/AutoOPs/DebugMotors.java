/*
This program was written by the FTC KTM #12529 team at the Polytechnic University in 2020. 
  
   @author Kolpakov Egor
*/
package org.firstinspires.ftc.teamcode.AutoOPs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "DebugMotors", group="AutoOP")
public class DebugMotors extends Robot {
    private ElapsedTime runtime = new ElapsedTime();
    private int time = 3000; //One way rotation time (ms)

    @Override
    public void runOpMode() {
        initHW(hardwareMap);
        BNO055IMU imu;
        Orientation angles;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        waitForStart();
        {
            telemetry.clear();
            runtime.reset();

            //Voltage regulation depending on the battery charge level
            double voltage = BatteryVoltage();
            double koeff = 13.0 / voltage;
            koeff = Math.pow(koeff, 1.25);
            imu.initialize(parameters);
            double time1;
            double time2;
            time1=getRuntime();
            time2=getRuntime();
            while(opModeIsActive()&&!isStopRequested()) {
                //m6Intake.setPower(-0.86);
                time2=getRuntime();
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Right sensor", DistanceSensor_right.getDistance(DistanceUnit.CM));
                telemetry.addData("Left sensor", DistanceSensor_left.getDistance(DistanceUnit.CM));
                telemetry.addData("Forward sensor", DistanceSensor_forward.getDistance(DistanceUnit.CM));
                telemetry.addData("angle1", angles.firstAngle);
                telemetry.addData("angle2", angles.secondAngle);
                telemetry.addData("angle3", angles.thirdAngle);
                telemetry.update();
            }
        }
    }
}
