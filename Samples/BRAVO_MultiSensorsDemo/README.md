
### Bravo Multi Sensors demo



**Features**

---

- Connect to LWM2M Portal
- Retrieve environment information with BSEC library sensor, Tampering and 3D vector rotation with BHI library sensors

---

#### Prerequisites on the module

This application requires the files **object_26242.xml**, **object_26250.xml** and **object_26251.xml** (provided) to be stored into module's `/mod/` folder, along with the application binary itself.

To load it, use

`AT#M2MWRITE=/mod/object_26242.xml,1358`
`AT#M2MWRITE=/mod/object_26250.xml,2249`
`AT#M2MWRITE=/mod/object_26251.xml,1971`

And at each prompt, send the file content in raw mode.

#### Prerequisites on the OneEdge Portal

This application requires the **object_26242.xml**, **object_26250.xml** and **object_26251.xml** content to be stored in your OneEdge organization object registry. The latter can be accessed from the link https://<server_url>/lwm2m/object_registry
where <server_url> could be for example `portal-dev.telit.com`. open the xml file in a notepad tool, select all the content and copy it. Then, in the object registry webpage, press "New Object" button on the right and paste the content of the xml file, then press Add button.

Now from Developer webpage, go in **Thing Definitions** page from the list on the left and press `Import` button on the right. Press `Attach File` and provide `json/bravo_MultiSensorsDemo_thing_def.json` from the project root, then press `Import`.

Again from the Developer webpage, select **Device Profiles**, `Import` button, `Attach File` and provide `json/bravo_MultiSensors_device_profile.json`, then press `Import`.


#### Local run

For testing purposes, it is possible to build the project without the LWM2M functionality. To do so, edit the [Makefile.in](Makefile.in) file at the line

```
LWM2M = 1
```

and set the variable to 0

```
LWM2M = 0
```




**External Libraries**

To build the application it is required to put `libalgobsec.ar` file into the project's BOSCH/BSEC folder. The library can be retrieved at the link
https://www.bosch-sensortec.com/software-tools/software/bsec/ . Download the BSEC 1.4.8.0 v3 version archive, then extract the library `libalgobsec.a` from the ZIP file and rename as `libalgobsec.ar`. It can be found in the archive directory
*BSEC_1.4.8.0_Generic_Release_updated_v3/algo/normal_version/bin/gcc/Cortex_A7/without_FPIC*


Please note: all the apps using BSEC library configure the device with the **18v3s_4d** option. If a different version of the library is in use, please replace the BOSCH/BME680/bsec_serialized_configurations_iaq.c file in the project with the one inside `BSEC_x.x.x.x_Generic_Release/config/generic_18v_3s_4d/` 


---


