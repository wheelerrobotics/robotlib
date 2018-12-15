package org.wheelerschool.robotics.robotlib.motion;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;

public class MotorGroup extends ArrayList<DcMotorSimple> {
    public void setPower(double power) {
        for (DcMotorSimple m : this) {
            m.setPower(power);
        }
    }
}
