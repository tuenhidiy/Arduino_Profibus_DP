# Arduino_Profibus_DP
I. PARTS LIST
Main component includes:
- 01pcs x DFRduino UNO R3 - Arduino Compatible: https://www.dfrobot.com/product-838.html
- 01pcs x DFRobot RS485 Shield for Arduino: https://www.dfrobot.com/product-1024.html
- 01pcs x DFRduino Ethernet Shield (Optional): https://www.dfrobot.com/product-455.html
- 01pcs x PLC S7-300.
- 01pcs x Relay 24VDC.
- 01pcs x Inductive sensor.
- 01pcs x Lamp 220VAC.
- 01pcs x Touch button with LEDs indicator.
- 03 meters x Profibus cable.
- 03pcs x Profibus connectors.

II. TESTING DESCRIPTION
The 220VAC lamp can be controlled in toogle mode by virtual and physical input as follows:
- Touch button is connected to Arduino plus RS485 shield and send the command to PLC S7-300 via Profibus-DP network at Virtual Input I0.0
- Inductive sensor is connected to PLC S7-300 directly at Physical Input I124.0
- 24VDC relay coil is connected to PLC Physical Output Q124.0 and lamp 220VAC is connected to N.O (normal open) contact of this relay. Even though the lamp is controlled by Arduino or PLC, Arduino receive the feedback of lamp status from PLC at physical output Q0.0 and display lamp status by red led (lamp 220VAC off) /green led (lamp 220VAC on) on touch button.
Check detail at: https://www.instructables.com/id/INTEGRATING-ARDUINO-INTO-PLC-SYSTEM/

YouTube link:

https://www.youtube.com/watch?v=TC1FKypClzU

https://www.youtube.com/watch?v=G7TGFvfsMPU

Please LIKE and SUBSCRIBE to my YouTube channel:
https://www.youtube.com/tuenhidiy

