
### Bravo SmartLock demo



**Features**

---

- Connect to LWM2M Portal
- Retrieve movement information from BMI160 sensor
- Update portal about current status (door status)

---

#### Prerequisites

This application requires the file **object_26247.xml** (provided) to be stored into module's `/mod/` folder, along with the application binary itself.

To load it, use 

`AT#M2MWRITE=/mod/object_26247.xml,1427`

And at prompt, send the file content in raw mode. 


#### Calibration
At application startup, the board red LED will turn ON. After it turns OFF, move the board to perform a door open movement. Wait 3 seconds, then perform a door close movement.
The calibration is now complete and the board will signal any door status change


---


