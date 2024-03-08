package frc.robot.utilities;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Map;
import java.util.HashMap;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj.Filesystem;

public class JSONManager {
    // Singleton Design Pattern
    private static JSONManager instance;
    public static JSONManager getInstance() {
        if (instance == null) {
            instance = new JSONManager();
        }
        return instance;
    }

    private final File DIRECTORY = Filesystem.getDeployDirectory();
    private final File DATAFILE;
    private final String LEFT_PIVOT_INDEX = "LEFT_PIVOT_MOTOR_POSITION";
    private final String RIGHT_PIVOT_INDEX = "RIGHT_PIVOT_MOTOR_POSITION";

    /** Creates a new JSON Manager object */
    public JSONManager() {
        DATAFILE = new File(DIRECTORY, "data.json");

        // Creates file in case it does not exist
        try {
            if (DATAFILE.createNewFile()) {
                savePivotPositions(Double.valueOf(0), Double.valueOf(0));
            }
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Writes the pivot's positions to the JSON file
     * 
     * @param leftMotorPosition
     * @param rightMotorPosition
     */ 
    public void savePivotPositions(double leftMotorPosition, double rightMotorPosition) {
        Map<String, Double> map = new HashMap<String, Double>();
        map.put(LEFT_PIVOT_INDEX, leftMotorPosition);
        map.put(RIGHT_PIVOT_INDEX, rightMotorPosition);
        JSONObject json = new JSONObject(map);
        
        try {
            FileWriter file = new FileWriter(DATAFILE);
            file.write(json.toJSONString());
            file.close();
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Writes the pivot's position to the JSON file (overloaded)
     * 
     * @param motorPosition
     */ 
    public void savePivotPositions(double motorPosition) {
        savePivotPositions(motorPosition, motorPosition);
    }

    /**
     * Get the pivot positions from the JSON file
     * @return an array with the left [0] and right [1] positions
     */
    public double[] getPivotPositions() {
        double[] positions = new double[2];
        try {
            FileReader file = new FileReader(DATAFILE);
            JSONObject json = (JSONObject) new JSONParser().parse(file);
            positions[0] = (Double) json.get(LEFT_PIVOT_INDEX);
            positions[1] = (Double) json.get(RIGHT_PIVOT_INDEX);
            file.close();
        }
        /* ClassCastException only happens when casting from Long to Double during json.get()
        because JSONObject turns 0 (default value when creating file) into a Long.
        `positions` has default values of 0, so it will return [0,0] anyways */ 
        catch (IOException | ParseException | ClassCastException e) {
            e.printStackTrace();
        }
        return positions;
    }
}
