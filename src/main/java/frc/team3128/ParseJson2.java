package frc.team3128;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ParseJson2 {

    HashMap<Integer, Pose3d> aprilTags = new HashMap<Integer, Pose3d>();
    List<AprilTag> aprilTagsList;
    double fieldwidth;
    double fieldlength;

    public ParseJson2(String filepath){

      Object obj = new Object();
      try {
        obj = new JSONParser().parse(new FileReader(filepath));
      } catch (FileNotFoundException e) {
        e.printStackTrace();
      } catch (IOException e) {
        e.printStackTrace();
      } catch (ParseException e) {
        e.printStackTrace();
      } 
        
      // typecasting obj to JSONObject 
      JSONObject jo = (JSONObject) obj; 
        
      JSONArray tags = ((JSONArray) jo.get("tags")); 

      JSONObject field = ((JSONObject) jo.get("field")); 
         

      for (int i = 0; i < tags.size(); i++) {
        JSONObject tag = (JSONObject) tags.get(i);
        JSONObject tagID = (JSONObject) tag.get("ID");
        JSONObject pose = (JSONObject) tag.get("pose");
        JSONObject translation = (JSONObject) pose.get("translation");
        JSONObject rotation = (JSONObject) pose.get("rotation");
        JSONObject quaternion = (JSONObject) rotation.get("quaternion");
        double rotationW = Double.parseDouble(quaternion.get("W").toString());
        double rotationX = Double.parseDouble(quaternion.get("X").toString());
        double rotationY = Double.parseDouble(quaternion.get("Y").toString());
        double rotationZ = Double.parseDouble(quaternion.get("Z").toString());
        Quaternion quaternion3d = new Quaternion(rotationW, rotationX, rotationY,rotationZ);
        double translationx = Double.parseDouble(translation.get("x").toString());
        double translationy = Double.parseDouble(translation.get("y").toString());
        double translationz = Double.parseDouble(translation.get("z").toString());
        Translation3d translation3d = new Translation3d(translationx, translationy, translationz);
        Pose3d aprilTagPose = new Pose3d(translation3d, new Rotation3d(quaternion3d));

        double fieldwidth = Double.parseDouble(field.get("width").toString());
        double fieldlength = Double.parseDouble(field.get("length").toString());
        this.fieldwidth = fieldwidth;
        this.fieldlength = fieldlength;
        // System.out.print(i + " " + translation.get("x"));
        aprilTags.put(i, aprilTagPose);
        aprilTagsList.add(new AprilTag(Integer.parseInt(tagID.toString()), aprilTagPose));
      }
    }

    public HashMap<Integer, Pose3d> returnHashMap(){
        return aprilTags;
    }

    public List<AprilTag> returnList(){
      return aprilTagsList;
    }

    public double returnFieldWidth(){
      return fieldwidth;
    }

    public double returnFieldLength(){
      return fieldlength;
    }
} 