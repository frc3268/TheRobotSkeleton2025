Naming:
- Subsystems: End with "Subsystem"
- Commands: End with "Command"
- Constants: UPPERCASE_WITH_UNDERSCORES
- Methods/Functions: startWithLowerCaseThenCamelCase
- Classes: CamelCase

Structure:
- Lib: All the library (ie non wpilib) things go here
- Robot: everything else goes here
Ideals:
- Assume as little as possible about the specs of the end user, make implementation possible with as little as possible
    - However, allow for use to the max if need be, provide useful and clean ways of doing so
- Allow for extension by user
- Prevent opacity in how things are done
- do the copying so that the user doesnt have to

Todo:
-Camera
-Testing
-auto + joystick chooser(dashboard in gen.)
-simulation
