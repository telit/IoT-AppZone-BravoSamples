
### Bravo Environment demo



**Features**

---

- Connect to LWM2M Portal
- Retrieve environment information with BSEC library sensor

---

#### Prerequisites

This application requires the file **object_26251.xml** (provided) to be stored into module's `/mod/` folder, along with the application binary itself.

To load it, use 

`AT#M2MWRITE=/mod/object_26251.xml,1971`

And at prompt, send the file content in raw mode. 

** External Libraries **

To build the application it is required to put `libalgobsec.ar` file into the project's BOSCH/BSEC folder. The library can be retrieved at the link
https://www.bosch-sensortec.com/software-tools/software/bsec/ . Download BSEC 1.4.7.4 version archive, then extract the library `libalgobsec.a` from the ZIP file and rename as `libalgobsec.ar`. It can be found in the directory
/BSEC_1.4.7.4_Generic_Release/algo/normal_version/bin/gcc/Cortex_A7/


---



