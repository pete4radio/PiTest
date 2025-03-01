Use a superloop to execute board tests with VSCode and the RPi
extension. This is tested on MacOS, Windows, Windows WSL, Raspberry Pi
Zero and 4, and Ubuntu.

I presume you have git. On windows

![](Pictures/cmake.png){width="5.2862in"
height="0.8583in"}

git clone <https://github.com/pete4radio/slink.git>

Open VS Code, add the extension for git

![](Pictures/gitextension.png){width="3.198in"
height="1.0937in"}

Add the Raspberry Pi pico extension

![](Pictures/rpiextension.png){width="3.5425in"
height="2.0835in"}

Open the slink folder

![](Pictures/openfolder.png){width="3.448in"
height="2.3228in"}

![](Pictures/slinkdirectory.png){width="2.9063in"
height="2.3228in"}

I often see it autonomously say

![](Pictures/openocdinstalled.png){width="4.7925in"
height="1.0209in"}

The can be a long while if you're not on a fast WiFi (for example, close
to the WiFi box)

![](Pictures/downloading.png){width="2.8543in"
height="0.5417in"}

Open the Raspberry Pi Pico extension

![](Pictures/rpiextmenu.png){width="2.3508in"
height="3.9547in"}Click "Debug Project"

Select the Serial Monitor terminal window

![](Pictures/serialmonitor.png){width="6.9252in"
height="0.4299in"}I2C scan and Pico2 blink is running...
