#!/bin/sh

echo "starting database backup"

wc -l /var/log/dump-mysql.log #shows number of file lines of the file "dump-mysql.log"

day="$(date +'%d-%m-%Y %H:%M:%S')"

dump_database="backupclassicmodels_${day}".bz2 #creates a file named backupcla...

echo "dump_database.sh started at ${day}" >> /var/log/dump-mysql.log #stores the output of "echo" in the logfile "dump-mysql.log"

sudo mysqldump -uanas --password --no-tablespaces classicmodels | bzip2 -c > /home/database_backup/${dump_database} #stores the "classicmodels" database in the file "dump_database"

echo "dump_database.sh finished at ${day}" >> /var/log/dump-mysql.log

echo "$(tail -20 /var/log/dump-mysql.log)" > /var/log/dump-mysql.log #keeps only 20 of log file lines

echo "Number of log file lines has been set to 20"

wc -l /var/log/dump-mysql.log

cd /home/database_backup && ls -tp | grep -v '/$' | tail -n +6 | xargs -I {} rm -- {} #keeps only 5 recent backups

#You can replace --password by -p"yourpassword for mysql", although it is not recommended and is insecure.
