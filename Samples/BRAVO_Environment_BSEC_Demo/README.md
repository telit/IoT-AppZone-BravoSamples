
### Bravo Environment demo



**Features**

---

- Connect to LWM2M Portal
- Retrieve environment information with BSEC library sensor

---

#### Prerequisites on the module

This application requires the file **object_26251.xml** (provided) to be stored into module's `/mod/` folder, along with the application binary itself.

To load it, use 

`AT#M2MWRITE=/mod/object_26251.xml,1971`

And at prompt, send the file content in raw mode. 

#### Prerequisites on the OneEdge Portal

This application requires the **object_26251.xml** content to be stored in your OneEdge organization object registry. The latter can be accessed from the link https://<server_url>/lwm2m/object_registry
where <server_url> could be for example `portal-dev.telit.com`. open the xml file in a notepad tool, select all the content and copy it. Then, in the object registry webpage, press "New Object" button on the right and paste the content of the xml file, then press Add button.

Now from Developer webpage, go in **Thing Definitions** page from the list on the left and press `Import` button on the right. Press `Attach File` and provide `json/bravo_EnvironmentalDemo_thing_def.json` from the project root, then press `Import`.

Again from the Developer webpage, select **Device Profiles**, `Import` button, `Attach File` and provide `json/bravo_EnvironmentalDemo_device_profile.json`, then press `Import`.


** External Libraries **

To build the application it is required to put `libalgobsec.ar` file into the project's BOSCH/BSEC folder. The library can be retrieved at the link
https://www.bosch-sensortec.com/software-tools/software/bsec/ . Download BSEC 1.4.7.4 version archive, then extract the library `libalgobsec.a` from the ZIP file and rename as `libalgobsec.ar`. It can be found in the directory
/BSEC_1.4.7.4_Generic_Release/algo/normal_version/bin/gcc/Cortex_A7/


---



