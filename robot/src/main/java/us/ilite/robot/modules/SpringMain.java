package us.ilite.robot.modules;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.context.ConfigurableApplicationContext;

import java.util.List;

@SpringBootApplication
public class SpringMain {

    public static void main(String[] args) {
        //Put this line in the main or the initial part of our application. Basically
        //where we would initialize the modules now. If we don't have args, we can either
        //fake them our or just set them to an empty array.
        ConfigurableApplicationContext context = SpringApplication.run(SpringMain.class, args);

        //Once we start the SpringApplication, all the Services and Components will be started.
        //The Components will be automatically (Autowired) loaded into the services that requested
        //them through the @Autowired annotation. Note that to get all the components that
        //were autowired, you have to get the exact instance of the Service (in this case ModuleLoader)
        //to get all the modules within. If you instantiate a new instance it will not have the
        //modules inside it.
        ModuleService bean = context.getBean(ModuleService.class);
        List<IModule> modules = bean.getModules();

        //Once you have a collection of IModule, you can pretty much do whatever you want. We can
        //do things like add methods to enable/disable, add a priority method to sort the collection,
        //etc. Also, we can create additional services that can specify the specific implementation they
        //want.
        for(IModule aModule : modules) {
            System.out.println(aModule.getClass());
        }
    }
}
