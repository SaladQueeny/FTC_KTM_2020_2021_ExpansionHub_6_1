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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVisionL;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "nothing", group = "AutoOP")
public class nothing extends Robot {
    private ElapsedTime runtime = new ElapsedTime();
    OpenCvInternalCamera phoneCam;
    EasyOpenCVVisionL pipeline;

    @Override
    public void runOpMode() {
        initHW(hardwareMap);
        BNO055IMU imu;
        Orientation angles;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        s4Kicker.setPosition(0);
        waitForStart();
        {
            imu.initialize(parameters);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            //Voltage regulation depending on the battery charge level
            double voltage = BatteryVoltage();
            double koeff = 13.0 / voltage;
            koeff = Math.pow(koeff, 1.25);
            double time1;
            double time2;
            time1=getRuntime();
            time2=getRuntime();
            while(!isStopRequested()&&time2-time1<.5){
                setMotorsPowerleft(0.3*koeff,-0.3*koeff,0.3*koeff,-0.3*koeff,angles, imu, 0);
                time2=getRuntime();
            }
            setMotorsPower(0,0,0,0);
            // The choice of the direction of movement depending on the number of rings

        }

    }
}
