package org.firstinspires.ftc.teamcode.src.SensorsAndSubsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.teamcode.src.TeleOp.GenericOpmoodeTemplate;

public class ColorTest extends GenericOpmoodeTemplate {

    public ColorRangeSensor backColor;
    public ColorRangeSensor frontColor;

    public enum RGBCameraColors {
        Red,
        Green,
        Blue,
        Alpha
    }

    public double[] getBackRGB() {
        return new double[]{backColor.red(), backColor.green(), backColor.blue()};
    }



    public void runOpMode() throws InterruptedException{

        while(opModeIsActive()){

            telemetry.addData("back_red", backColor.red());
            telemetry.addData("front_red", frontColor.red());
            telemetry.addData("back_green",backColor.green());
            telemetry.addData("front_green", frontColor.green());
            telemetry.addData("back_blue", backColor.blue());
            telemetry.addData("front_blue", frontColor.blue());
            telemetry.addData("back_alpha", backColor.alpha());
            telemetry.addData("front_alpha", frontColor.alpha());

            telemetry.addData("identity", CenterStageGameObject.identify());

        }
    }

}
