package org.wheelerschool.robotics.robotlib.motion;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.wheelerschool.robotics.robotlib.matrix.SingularValueDecomposition;

public class MechanumDrive {
    private DcMotor[] motors;
    private GeneralMatrixF thrustMotorCoeff;

    public MechanumDrive(DcMotor[] motors, float[][] mpos) {
        /**
         * @param motors: List of motors
         * @param mpos: List of [x, y, rot, xsign]
         *
         * NOTE: X axis sign corresponds to: +1 (CCW), -1 (CW)
         */
        this.motors = motors;

        boolean goodFmt = true;
        for (float[] m : mpos) {
            if (m.length < 4) {
                goodFmt = false;
                break;
            }
        }

        if ((mpos.length < this.motors.length) || !goodFmt) {
            throw new ArrayStoreException("Positions must be in shape '4 * motors.length'");
        }

        GeneralMatrixF tmat = new GeneralMatrixF(3, this.motors.length);
        for (int midx=0; midx<tmat.numCols(); midx++) {
            float[] mdata = mpos[midx];

            // Determine thrust angle from angle
            //  include offset, based on wheel 'sided-ness':
            double thrAng =  mdata[2] + (Math.PI/4)*mdata[3];
            tmat.put(0, midx, (float) Math.cos(thrAng));
            tmat.put(1, midx, (float) Math.sin(thrAng));

            double perpAng = Math.atan2(mdata[1], mdata[0]);
            double cDist = Math.sqrt(Math.pow(mdata[0], 2) + Math.pow(mdata[1], 2));

            tmat.put(2, midx, (float) (Math.cos(thrAng - perpAng) * cDist));
        }

        thrustMotorCoeff = (GeneralMatrixF) (new SingularValueDecomposition(tmat).getSolver().getInverse());
    }

    public VectorF calc(float x, float y, float rot) {
        VectorF thrustMat = new VectorF(x,y,rot);
        return this.thrustMotorCoeff.multiplied(thrustMat);
    }

    public void drive(VectorF thrust) {
        for (int i=0; i<motors.length; i++) {
            motors[i].setPower(thrust.get(i));
        }
    }

    public void calcDrive(float x, float y, float rot) {
        drive(calc(x, y, rot));
    }
}
