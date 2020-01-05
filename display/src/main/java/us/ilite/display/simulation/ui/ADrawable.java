package us.ilite.display.simulation.ui;


import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import javafx.scene.canvas.GraphicsContext;

public abstract class ADrawable {

    public abstract void draw(GraphicsContext gc, Pose2d pose, Translation2d aspectRatio);
    public void draw(GraphicsContext gc, Pose2d pose){}


}
