
### Bravo Rotation demo



**Features**

---

- Connect to LWM2M Portal
- Retrieve rotation information from BMI160 sensor
- Update portal about current status (X,Y,Z,W and accuracy values)

---

#### Prerequisites

This application requires the file **object_26250.xml** (provided) to be stored into module's `/mod/` folder, along with the application binary itself.

To load it, use 

`AT#M2MWRITE=/mod/object_26250.xml,2249`

And at prompt, send the file content in raw mode. 

#### Prerequisites on the OneEdge Portal

This application requires the **object_26250.xml** content to be stored in your OneEdge organization object registry. The latter can be accessed from the link https://<server_url>/lwm2m/object_registry
where <server_url> could be for example `portal-dev.telit.com`. open the xml file in a notepad tool, select all the content and copy it. Then, in the object registry webpage, press "New Object" button on the right and paste the content of the xml file, then press Add button.

Now from Developer webpage, go in **Thing Definitions** page from the list on the left and press `Import` button on the right. Press `Attach File` and provide `json/bravo_3D-RotationDemo_thing_def.json` from the project root, then press `Import`.

Again from the Developer webpage, select **Device Profiles**, `Import` button, `Attach File` and provide `json/bravo_3D-RotationDemo_device_profile.json`, then press `Import`.


---


