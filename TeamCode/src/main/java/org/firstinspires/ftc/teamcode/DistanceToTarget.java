package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class DistanceToTarget {
    double diffx;
    double diffy;
    double diffh;
    double vector;

    public DistanceToTarget (double diffx, double diffy, double diffh, double vector){
        this.diffx = diffx;
        this.diffy = diffy;
        this.diffh = diffh;
        this.vector = vector;
    }

    public DistanceToTarget(){
        diffx = 0;
        diffy = 0;
        diffh = 0;
        vector = 0;
    }

    public DistanceToTarget find(SparkFunOTOS.Pose2D pos, Target target)
    {
        diffx = target.x - pos.x;
        diffy = target.y - pos.y;
        diffh = target.h - pos.h;
        vector = Math.sqrt(diffx * diffx + diffy * diffy);
        return this;
    }

    public boolean closeEnough (DistanceToTarget dtt){
        return (Math.abs(dtt.diffx) < .5 && Math.abs(dtt.diffy) < .5 && Math.abs(dtt.diffh) < .5);
    }
}
