package us.ilite.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Deprecated
public class ParallelCommand implements ICommand {

  List<ICommand> mCommandList;
  
  public ParallelCommand(List<ICommand> pCommandList) {
    this.mCommandList = new LinkedList<>();
    this.mCommandList.addAll(pCommandList);
  }
  
  public ParallelCommand(ICommand ... pCommands ) {
    this(Arrays.asList(pCommands));
  }
  
  @Override
  public void init(double pNow) {
    for(ICommand c : mCommandList) {
      c.init(pNow);
    }
  }

  @Override
  public boolean update(double pNow) {
    List<ICommand> toremove = new ArrayList<>();
    for(ICommand c : mCommandList) {
      if(c.update(pNow)) {
        toremove.add(c);
      }
    }
    mCommandList.removeAll(toremove);
    if(mCommandList.isEmpty()) {
      return true;
    }
    return false;
  }

  @Override
  public void shutdown(double pNow) {
    for(ICommand c : mCommandList) {
      c.shutdown(pNow);
    }
  }

}
