
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


### Requirements

**External Libraries**

To build the application it is required to put `libalgobsec.ar` file into the project's BOSCH/BSEC folder. The library can be retrieved at the link
https://www.bosch-sensortec.com/software-tools/software/bsec/ . Download BSEC 1.4.7.4 version archive, then extract the library `libalgobsec.a` from the ZIP file and rename as `libalgobsec.ar`. It can be found in the directory
/BSEC_1.4.7.4_Generic_Release/algo/normal_version/bin/gcc/Cortex_A7/

#### Simulated data
To run the code on a generic ME910C1 device, it is possible to build the code disabling the Bosch related functionalities. To do so, please refer to [Makefile.in](Makefile.in) file and edit the BOSCH_BSEC variable as below.

```
BOSCH_BSEC = 0
```
This will disable all Bosch dependencies and build the app with a simplified logic, simulating sensors data.

---



