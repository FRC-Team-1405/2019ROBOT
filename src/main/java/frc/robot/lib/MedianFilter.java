package frc.robot.lib;

import java.util.Arrays;
import java.util.stream.DoubleStream;

public class MedianFilter {
    double values[];
    protected int position = 0; 
    protected boolean isInitialized = false;

    public MedianFilter(int size){
        values = new double[size];
        position = 0;
    }

    public void reset(){
        isInitialized = false;
    }

    public double filter(double value){
        if (!isInitialized){
            Arrays.fill(values, value);
            position = 0;
            isInitialized = true; 
        }
        
        values[position++ % values.length] = value;
        return DoubleStream.of(values)
                     .sorted()
                     .skip(values.length/2)
                     .findFirst()
                     .getAsDouble();
    }
}
