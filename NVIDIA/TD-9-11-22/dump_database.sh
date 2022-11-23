#!/bin/sh

echo "starting database backup"

wc -l /var/log/dump-mysql.log #shows number of file lines

day="$(date +'%d-%m-%Y %H:%M:%S')"

dump_database="backupclassicmodels_${day}".bz2

echo "dump_database.sh started at ${day}" >> /var/log/dump-mysql.log

sudo mysqldump -uanas -panasderkaoui --no-tablespaces classicmodels | bzip2 -c > /home/database_backup/${dump_database}

echo "dump_database.sh finished at ${day}" >> /var/log/dump-mysql.log

echo "$(tail -20 /var/log/dump-mysql.log)" > /var/log/dump-mysql.log #keeps only 20 of log file lines

echo "Number of log file lines has been set to 20"

wc -l /var/log/dump-mysql.log

cd /home/database_backup && ls -tp | grep -v '/$' | tail -n +6 | xargs -I {} rm -- {}
