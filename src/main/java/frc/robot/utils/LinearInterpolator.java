package frc.robot.utils;
import java.util.ArrayList;

import edu.wpi.first.math.Pair;

public class LinearInterpolator {
    ArrayList<Pair<Double,Double>> list;

    public LinearInterpolator(ArrayList<Pair<Double,Double>> list){
        this.list = list;
    }    

    public double get(double value){
        var prev = new Pair<>(0.0, 0.0);
        for(Pair<Double,Double> item : this.list){
            if(item.getFirst() >= value){
                // y = mx + b
                var slope = (item.getSecond() - prev.getSecond()) / (item.getFirst() - prev.getFirst());
                return (slope * value); 
            }
        }
        return list.get(list.size()-1).getSecond();
        //todo: actually try at getting an interpolation for farther points

    }
}
