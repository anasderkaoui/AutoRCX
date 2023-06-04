After the final report from the previous sessions, you may have seen that I bricked my Nvidia Jetson Nano card because of some faulty commands.
Today, I succeeded at reflashing it and bringing it back to its normal state.
Here are the steps that I have followed to unbrick it:<br >

1- Follow the steps described in **[this document](https://auvidea.eu/download/Software)**, note that I have an Auvidea carrier board. In this document you will find the step by step guide to reflash the jetson nano.<br >

2- You may encounter some problems as I did. If so, here are the problems to the problems I had while flashing the card:<br >
   - ERROR 8 : The card did not successfuly boot into recovery mode. You will have to check connection between the jumpers used and the card.<br >
   - ERROR 1 : It has something to do with system cache. All you need to do is restart the laptop you are using to flash the card and change the cable connecting the jetson nano to the laptop.<br >

After this, the card should be now good to go. You will need an external monitor and a keyboard to reconfigure it to its original state.<br >

Finally, I want to add one problem that I ran into while trying to move user data from /home to /media/storage/home as explained in [the 19th session's report](https://github.com/anasderkaoui/AutoRCX/blob/main/My%20project%20reports/19th%20session's%20report.md?plain=1).<br >
While following the steps described in that session, I added **<>** to the UUID number that was assigned to my disk partition. After that, the card booted in emergency mode and did not want to boot normaly even after unplugging and plugging back the power cable. I needed an external monitor and a keyboard to figure this out.

![rn_image_picker_lib_temp_26d85087-8dd3-40f0-8c19-2a6d464f14e2](https://github.com/anasderkaoui/AutoRCX/assets/115218309/e5c9e3f2-345b-4de2-83d1-1c52e39cb387)

After searching, it turns out that these signs **<>** were the problem.
After the card had booted in emergency mode, I pressed the ENTER key to continue.

![rn_image_picker_lib_temp_004da67b-501c-4a24-9669-6b4906c3dffc](https://github.com/anasderkaoui/AutoRCX/assets/115218309/bc8e8c5a-f585-40f8-8cce-8a6d717d5084)

Then I simply modified the file that contained the UUID number and deleted the <> signs.<br >

![rn_image_picker_lib_temp_d4bf532e-9744-44f4-9b78-a2ac7e8fbe27](https://github.com/anasderkaoui/AutoRCX/assets/115218309/4a1c61cc-eb78-429a-bc4c-4b4afcc25490)

Eventually, I had to restart the card and the problem was solved !

**As a bonus, I will guide you through two methods, one being more effective than the other, to get the output of your Jetson Nano on your laptop screen without using any external peripherals !**

1- In the first method we will be using VNC (Virtual Network Computing) (The least effective method because the output is not very smooth). In this method you will need a monitor, a keyboard and if you want also a mouse.
  - First, after booting up the system, you will need to enter this command line in the terminal: `sudo vim /usr/share/glib-2.0/schemas/org.gnome.Vino.gschema.xml`
    This command will open up a file and we will add a key to it.
  - Add this key as shown in the screenshot below: `<key name=’enabled’ type=’b’>`
                  `<summary>Enable remote access to the desktop</summary>`
                  `<description>`
                  `If true, allows remote access to the desktop via the RFB`
                  `protocol. Users on remote machines may then connect to the`
                  `desktop using a VNC viewer.`
                  `</description>`
                  `<default>true</default>`
                  `</key>`
                  
                  ![image](https://github.com/anasderkaoui/AutoRCX/assets/115218309/b36faae1-847f-4dcd-9b02-b3b4917d414a)

  - After that, we will need to compile the file. Run this command to do so: `sudo glib-compile-schemas /usr/share/glib-2.0/schemas`
  - Now search for "Desktop Sharing" in the search bar (can be accessed by pressing the "Windows" key) and open it.
  - Tick the “Allow other users to view your desktop” and also “Allow other users to control your desktop” checkmarks. Then make sure “You must confirm each access to this machine” is turned off. Finally tick the “Require the user to enter this password” checkmark, and enter a password for the VNC session.

![image](https://github.com/anasderkaoui/AutoRCX/assets/115218309/ae0dbcc5-94e7-4b36-8f4d-0546ee78433a)

  - Open "startup applications" using the search bar. Now, click Add at the right of the box, then type ‘Vino’ in the name box, and then in the command box enter **/usr/lib/vino/vino-server**. Click Save at the bottom right of the box, and then close the app.

![image](https://github.com/anasderkaoui/AutoRCX/assets/115218309/70dab529-da0d-4bf6-b746-f0bb0e2191e0)

  - Next, we need to disable encryption of the VNC connection to get things working. To do this, open the terminal and enter the following commands: `gsettings set org.gnome.Vino require-encryption false`
`gsettings set org.gnome.Vino prompt-enabled false`
