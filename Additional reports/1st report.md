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

![rn_image_picker_lib_temp_d4bf532e-9744-44f4-9b78-a2ac7e8fbe27](https://github.com/anasderkaoui/AutoRCX/assets/115218309/8d029d64-1bea-40ac-a528-4d566d981abb)

Eventually, I had to restart the card and the problem was solved !
