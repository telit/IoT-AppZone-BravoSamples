
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

Please refer to the [Time Series App Note](https://github.com/telit/oneedge-projects-resources/blob/main/use-cases/time-series/Docs/80654NT11932A_OneEdge_Use_Case_Time-series_r0.pdf)



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


