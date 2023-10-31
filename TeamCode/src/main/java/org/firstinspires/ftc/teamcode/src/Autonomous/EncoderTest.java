package org.firstinspires.ftc.teamcode.src.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = " AUTO")
public class EncoderTest extends Encoder {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        DriveForward(.8, 35);
        StopDriving();
        sleep(300);
        TurnRight(.8, 710);
        StopDriving();
        sleep(300);
    }
}
