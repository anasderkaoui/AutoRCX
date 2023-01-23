# Setup to manage the content of the log file of a script
<br />

## Managing the log file of "dump-database.sh"
<br />

I already have the "dump-database.sh" script in the repository.<br />
I programmed the file to only keep the 20 recent files in the log file. You can do so by typing, in the script, the commands :<br />

1 - `wc -l /var/log/dump-mysql.log`, this command just shows how many lines the log file has.<br />
2 - `echo "$(tail -20 /var/log/dump-mysql.log)" > /var/log/dump-mysql.log`, this keeps only the last 20 lines of the logfile, thus keeping the file size small. You can choose to rerun the previous command to see how many lines are still in the logfile, you should have 20.<br />

This is one solution to manage the size of the logfile of the script. If you want to go further, you can compress the logfile by trying to run the `"truncate -s 800MB /var/log/yourlogfile"` command. This command will shrink your logfile size to 800MB (for example) if the file size was bigger than that (in this case the "yourlogfile" file size was 1,2Gb and after running the command the size went down to 763Mb).