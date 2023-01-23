
## This is a full guide on how to :

 - Install MySQL on Linux (in my case I run Linux in my VM)
 - Create a database from this zip file : https://www.mysqltutorial.org/wp-content/uploads/2018/03/mysqlsampledatabase.zip
 - Write a script that backs up the classicmodels database, complying with the following requirements:
    * The database backup is contained in a single file
    * This file is compressed in .bz2 format
    * The file is timestamped (year-month-day-hour-minutes-seconds)
    * If the backup is successful, the script keeps only the 5 most recent backup files
    * This script runs every 10 minutes, logs are sent to /var/log/dump-mysql.log
    * There is an option to control the size of the log file /var/log/dump-mysql.log.


 ## setup-mysql :
    * Explains how to install MySQL and how to import database

 ## dump-database.sh :
    * The database backup script

 ## dump-database.service & dump-database.timer : 
    * Files for configuring backup as a service

 ## setup-dump-database-service : 
    * Script explaining how to configure the service

 ## setup-dump-database-logs-cleanup :
    * Script explaining how to configure log monitoring
