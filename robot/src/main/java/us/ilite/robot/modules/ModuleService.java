package us.ilite.robot.modules;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.ArrayList;
import java.util.List;

@Service
public class ModuleService {

    /**
     * This will be given all of the IModules that have the @Component annotation.
     *
     * When we get to unit testing, we can specify whatever we want to pass in here in a
     * seperate constructor
     */
    @Autowired
    private List<IModule>modules = new ArrayList<>();

    public List<IModule> getModules() {
        return modules;
    }
}
