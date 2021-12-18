package us.ilite.robot.modules;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;

@SpringBootApplication
public class SpringMain {

    public static void main(String[] args) {
        //Put this line in the main or the initial part of our application. Basically
        //where we would initialize the modules now. If we don't have args, we can either
        //fake them our or just set them to an empty array.
        SpringApplication.run(SpringMain.class, args);
    }
}
