
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

#### Prerequisites on the OneEdge Portal

This application requires the **object_26242.xml** content to be stored in your OneEdge organization object registry. The latter can be accessed from the link https://<server_url>/lwm2m/object_registry
where <server_url> could be for example `portal-dev.telit.com`. open the xml file in a notepad tool, select all the content and copy it. Then, in the object registry webpage, press "New Object" button on the right and paste the content of the xml file, then press Add button.

Now from Developer webpage, go in **Thing Definitions** page from the list on the left and press `Import` button on the right. Press `Attach File` and provide `json/bravo_TamperDemo_thing_def.json` from the project root, then press `Import`.

Again from the Developer webpage, select **Device Profiles**, `Import` button, `Attach File` and provide `json/bravo_TamperDemo_device_profile.json`, then press `Import`.

Lastly, from the Developer webpage, select **Triggers**, `Actions` menu on the right, `Import` , `Attach File` and provide `json/bravo_TamperDemo_triggers.json`, then press `Import`. Now open the trigger **bravo_TamperDemo_state_trigger** by pressing the View button (the eye icon on the left) and be sure to press the `play` button, and that the trigger status is 'started'.

#### Local run

For testing purposes, it is possible to build the project without the LWM2M functionality. To do so, edit the [Makefile.in](Makefile.in) file at the line

```
LWM2M = 1
```

and set the variable to 0

```
LWM2M = 0
```



