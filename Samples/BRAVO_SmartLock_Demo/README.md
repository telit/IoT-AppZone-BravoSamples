
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

#### Prerequisites on the OneEdge Portal

This application requires the **object_26247.xml** content to be stored in your OneEdge organization object registry. The latter can be accessed from the link https://<server_url>/lwm2m/object_registry
where <server_url> could be for example `portal-dev.telit.com`. open the xml file in a notepad tool, select all the content and copy it. Then, in the object registry webpage, press "New Object" button on the right and paste the content of the xml file, then press Add button.

Now from Developer webpage, go in **Thing Definitions** page from the list on the left and press `Import` button on the right. Press `Attach File` and provide `json/bravo_SmartLock_thing_def.json` from the project root, then press `Import`.

Again from the Developer webpage, select **Device Profiles**, `Import` button, `Attach File` and provide `json/bravo_SmartLock_device_profile.json`, then press `Import`.

Lastly, from the Developer webpage, select **Triggers**, `Actions` menu on the right, `Import` , `Attach File` and provide `json/bravo_SmartLock_triggers.json`, then press `Import`. Now open the trigger **bravo_SmartLockDemo_state_trigger** by pressing the View button (the eye icon on the left) and be sure to press the `play` button, and that the trigger status is 'started'.

#### Calibration
At application startup, the board red LED will turn ON. After it turns OFF, move the board to perform a door open movement. Wait 3 seconds, then perform a door close movement.
The calibration is now complete and the board will signal any door status change


---


