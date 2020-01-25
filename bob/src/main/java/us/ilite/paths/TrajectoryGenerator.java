package us.ilite.paths;

import com.team319.io.ConfigExporter;
import com.team319.io.ConfigImporter;
import com.team319.ui.BobTrajectoryApp;

import java.io.File;
import java.io.IOException;

public class TrajectoryGenerator {

    public static void main(String[] pArgs) throws IOException {
        File f = new File("config.txt");
        if(!f.exists()) {
            System.err.println("Copy the default config file from bob trajectory.");
            System.exit(-1);
        }
        File parent = f.getAbsoluteFile().getParentFile();
        ConfigExporter.setConfigFolder(parent);
        System.out.println("Loading " + parent.getAbsolutePath());
        ConfigImporter.importConfig(parent);

        System.out.println("Running BobTrajectory");
        new BobTrajectoryApp();
    }

}