
### Bravo Tampering demo



**Features**

---

- Connect to LWM2M Portal
- Retrieve movement information from BMI160 sensor
- Update portal about current status (IDLE, TAMPER, WALKING... )

---

#### Prerequisites

This application requires the file **object_26242.xml** (provided) to be stored into module's `/mod/` folder, along with the application binary itself.

To load it, use 

`AT#M2MWRITE=/mod/object_26242.xml,1358`

And at prompt, send the file content in raw mode. 

---


