package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="Robot Class Test", group="Linear Opmode")
public class Test extends LinearOpMode {
    // Declare OpMode members.
   private Robot robot = new Robot(this);
    @Override
    public void runOpMode() {
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.attachAll();

            robot.mechLeftSlide(50, 100);
        }
    }
}
