
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


