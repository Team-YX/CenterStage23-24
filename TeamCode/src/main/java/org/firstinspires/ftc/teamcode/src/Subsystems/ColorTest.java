package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.TeleOp.GenericOpmoodeTemplate;

@TeleOp(name = "ColorTest")
public class ColorTest extends GenericOpmoodeTemplate {


    public enum RGBCameraColors {
        Red,
        Green,
        Blue,
        Alpha
    }

    public double[] getBackRGB() {
        return new double[]{back_color.red(), back_color.green(), back_color.blue()};
    }

    public double[] getFrontRGB() {
        return new double[]{front_color.red(), front_color.green(), front_color.blue()};
    }

    public CenterStageGameObject identifyBackContents() {
        return CenterStageGameObject.identify(this.getBackRGB());
    }


    public void runOpMode() throws InterruptedException{

        while(opModeIsActive()){

            telemetry.addData("back_red", back_color.red());
            telemetry.addData("front_red", front_color.red());
            telemetry.addData("back_green",back_color.green());
            telemetry.addData("front_green", front_color.green());
            telemetry.addData("back_blue", back_color.blue());
            telemetry.addData("front_blue", front_color.blue());
            telemetry.addData("back_alpha", back_color.alpha());
            telemetry.addData("front_alpha", front_color.alpha());

            telemetry.addData("back_identity", CenterStageGameObject.identify(getBackRGB()));
            telemetry.addData("front_identity", CenterStageGameObject.identify(getFrontRGB()));

            telemetry.addData("LEDBackColor", CenterStageGameObject.getLEDColorFromItem(
                    CenterStageGameObject.identify(getBackRGB())));
            telemetry.addData("LEDFrontColor", CenterStageGameObject.getLEDColorFromItem(
                    CenterStageGameObject.identify(getFrontRGB())));

        }
    }

}
