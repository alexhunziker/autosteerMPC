# Closed loop testing

Setting up the test vehicle and initiate a test case works as following:

1. Power supply for controller:
   - Connect the Raspberry Pi with the big battery pack on the vehicle (2.1A)
   - Connect the Lidar to the small white battery
2. Power supply for vehicle:
   - Connect the developer board (Servos, Motor Driver) to the small black battery pack (2A)
   - Connect the battery of the motor
3. Start the script:
   - Connect to the Pi via SSH
   - navigate to the controller folder
   - if needed: enter start and destination coordinates to the coordinator.py script
   - execute coordinator.py
4. Supply engine with power:
   - Flip the switch on the vehicle

## Options

- If the controller should be used standalone, switch mock_actuator to True
- If no lane detection should be computed, switch use_cv to False
- If debug logs should be shown, set debug to True
