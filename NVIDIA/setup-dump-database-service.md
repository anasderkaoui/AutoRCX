# Setup service to run a a script
<br />

## The script I will run is named "dump_database.sh"
<br />

After the script is done (you can check it out in the repository), it should not return any errors for the service to run perfectly.
Setting up the service, run the follwong commands :<br />

1 - `sudo gedit /etc/systemd/system/dump-database.service`, this creates the service "dump-database.service". If you have an error while running "gedit", you may have to download gedit by typing :
`sudo apt-get update`<br />
`sudo apt-get install gedit`<br />
`sudo apt-get update`<br />

2 - A window will open, then you will add your service configuration. Type the following lines :<br />

`[Unit]`<br />
`Description=Start database backup`<br />

`[Service]`<br />
`WorkingDirectory=/directory`<br />
`Type=simple`<br />
`ExecStrat=/directory/dump-database.sh`<br />

You can replace dump-database.sh by the script you want to run, but don't forget to modifiy the directory !<br />

Next, save the file and type the following commands :<br />

3 - `sudo systemctl daemon-reload`, this will rerun all generators, reload all unit files, and recreate the entire dependency tree.<br/>

4 - `sudo systemctl start dump-database.service`, this command starts the service. To see the status of the service type in :<br />
`sudo systemctl status dump-database.service`.<br />
Now you have successfully created the service to run.
