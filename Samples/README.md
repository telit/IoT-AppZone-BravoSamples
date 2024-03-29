

# AppZone m2mb Sample Apps 



Package Version: **1.0.13**

Firmware Version: **30.01.XX0.0**


## Features

This package goal is to provide sample source code for Bravo EVK kickstart.
 


## Deployment Instructions

### Binaries releases

The sample applications are available as source code, so they can be built with user's preferences. For testing purposes, binary releases are available at the link https://github.com/telit/IoT-AppZone-BravoSamples/releases. For each release, a set of archive files is provided, with the following naming convention:

`bravo_sample_apps_<apps_version>_<module_firmware_version>_<LOG_OUTPUT_CHANNEL>.zip.`

For example, **bravo_sample_apps_1.0.10_30_01_XX0_USB0.zip** will contain the binary files for Bravo Samples version 1.0.10, built for ME910C1 firmware 30.01.xx0, with logs output on USB0 port.

Each archive has the following structure:
 - `<ApplicationName>` Folder
   - `m2mapz.bin` file
 - `<ApplicationName>` Folder
   - `m2mapz.bin`
 ...

--- 

To manually deploy the Sample application on the devices perform the following steps:

1. Have **30.01.XX0.0** FW version flashed (`AT#SWPKGV` will give you the FW version)

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


## Applications 


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


MultiSensors Demo application. Debug prints on **MAIN UART**




### Bravo LED demo



**Features**

---

- Connect to LWM2M Portal
- Register the instances for LEDs
- React to value modification of object 3311 resource 5850 (On/Off) value from the OneEdge portal 
- Restore LEDs status at startup according to the LWM2M values

---

#### Prerequisites

This application requires the file **object_3311.xml** (provided) to be stored into module's `/mod/` folder, along with the application binary itself.

To load it, use 

`AT#M2MWRITE=/mod/object_3311.xml,3734`

And at prompt, send the file content in raw mode. 

#### Prerequisites on the OneEdge Portal

This application requires the **object_3311.xml** content to be stored in your OneEdge organization object registry. The latter can be accessed from the link https://<server_url>/lwm2m/object_registry
where <server_url> could be for example `portal-dev.telit.com`. open the xml file in a notepad tool, select all the content and copy it. Then, in the object registry webpage, press "New Object" button on the right and paste the content of the xml file, then press Add button.

---


LED management through IPSO object 3311 Demo application. Debug prints on **MAIN UART**




### Bravo LwM2M Time Series demo



**Features**

---

- Connect to LWM2M Portal
- Retrieve environment information with BSEC library sensor, Tampering and 3D vector rotation with BHI library sensors
- Push data to OneEdge portal as time series using Opaque resources with dedicated objects

---

#### Prerequisites on the module

This application requires the files **object_32001.xml** and **object_32002.xml** (provided) to be stored into module's `/XML/` folder, along with the application binary itself.

To load them, use

`AT#M2MWRITE=/XML/object_32001.xml,2272`
`AT#M2MWRITE=/XML/object_32002.xml,2365`



And at each prompt, send the file content in raw mode.

#### Prerequisites on the OneEdge Portal

Please refer to the **80654NT11932A_OneEdge_Use_Case_Time-series** App Note. It can be obtained from the Telit [Download Zone](https://www.telit.com/support-training/download-zone/) or by requesting it to ts-oneedge@telit.com.


#### Simulated data
To run the code on a generic ME910C1 device, it is possible to build the code disabling the Bosch related functionalities. To do so, please refer to [Makefile.in](Makefile.in) file and edit the BOSCH_BSEC variable as below.

```
BOSCH_BSEC = 0
```

This will disable all Bosch dependencies and build the app with a simplified logic, simulating sensors data.


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


Time Series Demo application. Debug prints on **MAIN UART**




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



Tampering Demo application. Debug prints on **MAIN UART**




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


#### Local run

For testing purposes, it is possible to build the project without the LWM2M functionality. To do so, edit the [Makefile.in](Makefile.in) file at the line

```
LWM2M = 1
```

and set the variable to 0

```
LWM2M = 0
```



Rotation Demo application. Debug prints on **MAIN UART**


