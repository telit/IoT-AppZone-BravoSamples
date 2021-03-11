

# AppZone m2mb Sample Apps 



Package Version: **1.0.6**

Firmware Version: **30.00.XX8**


## Features

This package goal is to provide sample source code for Bravo EVK kickstart.
 


## Deployment Instructions


To manually deploy the Sample application on the devices perform the following steps:

1. Have **30.00.XX8** FW version flashed (`AT#SWPKGV` will give you the FW version)

1. Copy _m2mapz.bin_ to _/mod/_ 
	```
	AT#M2MWRITE="/mod/m2mapz.bin",\<size\>,1
	```
1. Run `AT#M2MRUN=2,m2mapz.bin`
1. Run `AT+M2M=4,10`



## Known Issues



## Troubleshooting

* Application does not work/start:
	+ Delete application binary and retry
    ```
    AT#M2MDEL="/mod/m2mapz.bin"
    ```
	+ Delete everything, reflash and retry
    ```
    AT#M2MDEL="/mod/m2mapz.bin"
    AT#M2MDEL="/mod/appcfg.ini"
    ```
      
* Application project does not compile
	+ Right click on project name
	+ Select Properties
	+ Select AppZone tab
	+ Select the right plugin (firmware) version
	+ Press "Restore Defaults", then "Apply", then "OK"
	+ Build project again

---

## Making source code changes

### Folder structure

The applications code follow the structure below:

* `hdr`: header files used by the application
    * `app_cfg.h`: the main configuration file for the application
* `src`: source code specific to the application
* `BOSCH`: sources to manage Bosch sensors and devices mounted on the Bravo EVK board
* `azx`: helpful utilities used by the application (for GPIOs, LOGGING etc)
    * `hdr`: generic utilities' header files
    * `src`: generic utilities' source files
* `Makefile.in`: customization of the Make process


##Applications 


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



Environment Demo application. Debug prints on **MAIN UART**




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


** External Libraries **

To build the application it is required to put `libalgobsec.ar` file into the project's BOSCH/BSEC folder. The library can be retrieved at the link
https://www.bosch-sensortec.com/software-tools/software/bsec/ . Download BSEC 1.4.7.4 version archive, then extract the library `libalgobsec.a` from the ZIP file and rename as `libalgobsec.ar`. It can be found in the directory
/BSEC_1.4.7.4_Generic_Release/algo/normal_version/bin/gcc/Cortex_A7/


---



MultiSensors Demo application. Debug prints on **MAIN UART**




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


Rotation Demo application. Debug prints on **MAIN UART**




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

---


Tampering Demo application. Debug prints on **MAIN UART**


