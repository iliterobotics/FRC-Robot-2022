
To get started, clone and run this from git bash: `./gradlew clean assemble`

Please be sure to install the Microsoft VC distributable at the following link:
https://support.microsoft.com/en-us/help/2977003/the-latest-supported-visual-c-downloads

To run unit tests: `./gradlew :robot:test`

Edit your `.gitconfig` file, usually located in your home directory (e.g. `~/.gitconfig` or `c:\Users\JesseK\.gitconfig`). Add the following lines. This will help the mentors help you!
```
[alias]
           graph = log --graph --abbrev-commit --decorate --pretty --oneline --date=relative --all
           timeline = log --graph --abbrev-commit --decorate --format=format:'%C(bold blue)%h%C(reset) - %C(bold green)(%ar)%C(reset) %C(white)%s%C(reset) %C(dim white)- %an%C(reset)%C(bold yellow)%d%C(reset)' --all
           follow = log --graph --abbrev-commit --decorate --pretty --oneline --date=relative --all --follow
           history = log --date=short --graph --abbrev-commit --decorate --format=format:'%C(bold blue)%h%C(reset) - %C(bold green)(%ad)%C(reset) %C(white)%s%C(reset) %C(dim white)- %an%C(reset)%C(bold yellow)%d%C(reset)' --all --follow
```

IF you installed WPILib, edit your `.bashrc` file, usually located in your home directory.
(e.g. `~/.bashrc` or `c:\Users\JesseK\.bashrc`). Add the following line.
```
export JAVA_HOME='/c/Users/Public/wpilib/2020/jdk/bin'
```
To start the WPILib sim and see ShuffleBoard results:
1. Install WPILib. You do not need to install VSCode, but you DO need WPILib. You do not need the FRC Driver's Station.
2. Compile your code
3. `./gradlew ShuffleBoard simulateJava` (note you can also set these up in IntelliJ to run it from there)
4. IF for some reason the sim does not start, but a command prompt appears, in that command prompt type `robot\build\gra`, hit `[TAB]` so the file auto-completes, then hit `[ENTER]`
