Use a superloop to execute board tests with VSCode and the RPi
extension. This is tested on MacOS, Windows, Windows WSL, Raspberry Pi
Zero and 4, and Ubuntu.

Do you have git?  On windows I like the bash terminal:

![](Pictures/windowsgit.png)

and download cmake if you don't already have it.

git clone <https://github.com/pete4radio/PiTest.git>

Open VS Code, add the extension for git

![](Pictures/gitextension.png)

Add the Raspberry Pi pico extension

![](Pictures/rpiextension.png)

Open the PiTest folder

![](Pictures/openfolder.png)


I often see it autonomously say

![](Pictures/openocdinstalled.png)

The can be a long while if you're not on a fast WiFi (for example, close
to the WiFi box)

![](Pictures/downloading.png)

Open the Raspberry Pi Pico extension

![](Pictures/rpiextmenu.png)

Click "Debug Project"

Select the Serial Monitor terminal window

![](Pictures/serialmonitor.png)
I2C scan and Pico2 blink is running...
