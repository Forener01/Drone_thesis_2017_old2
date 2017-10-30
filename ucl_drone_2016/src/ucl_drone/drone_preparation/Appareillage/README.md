# AUTOCONFARPARROT README

## Configuration

#### Computer

For each drone, create a file with the `essid` of the drone as filename (with no suffix or extension), edit this file as a text file and write the `ip` address you want to allocate.

**Example:**

`drone_preparation/Appareillage/ardrone2_v2.4.8` contains:
```
192.168.1.15

```

#### Router

See Rowier documentation

(Router ip address `192.168.1.254`)

## Usage

0. Verify the computer has a wired connection to the WiFi router.
1.  Plug fully charged batteries in each drone and wait until four green LED's are on for each drone
2.	On the Ubuntu desktop, turn on the network applet, wait until the drone WiFi discovery of each drone
3.	In a console, run

    ```
    $ cd src/ucl_drone/drone_preparation/Appareillage
    $ bash autoconfarparrot
    ```

    Wait until success messages (in green) for each done.
    If a red message (error) is diplayed, follow instructions on the screen.
