# 5285 Training - Testbed Motor

## Task
Make the motor (Kraken X44) spin.
- When you press X on the controller, it turns counterclockwise 360 degrees.
- When you press B on the controller, it turns clockwise 360 degrees.
- The motor shaft always ends up at the same position.
- Make it as fast as possible.
- Robot telemetry with Sendable - make the motor rotations viewable on the driver dashboard.

## Project Instructions
- Only work in your own branch.
- Use Command-Based Programming.
- You may work together, but everyone has to have their own working code.

## Information
- Only the files MotorSubsystem.java and Constants.java need to be modified. Everything else is set up already.
- The Kraken library (Phoenix 6) has already been installed as a vendor dependency.
- Google, WPILib Docs, Phoenix 6 Docs, and their respective Java API Docs are helpful resources. Make sure you are on the "stable" docs (bottom right hand corner of web pages).

## Code Testing Instructions
1. Transfer Code
    - Commit code on your computer to your branch
    - Push the code
    - Go to team computer
    - Open WPILib VS Code
    - Checkout to your branch
    - Pull the code
2. Connect computer
    - Open Driver Station
    - Connect computer to roborio (USB B cable)
    - Plug in controller to computer
    - Plug in battery to test board
    - Turn on roborio
    - Check Driver Station - "Communications" green
    - VS Code: Three buttons on top right -> Deploy Code
    - Wait for Driver Station "Robot Code" green
3. Test
    - Press "Enable" on Driver Station
    - If something bad happens, press spacebar (emergency stop) or enter (disable)