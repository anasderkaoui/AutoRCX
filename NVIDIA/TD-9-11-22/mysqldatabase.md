# MySQL setup

## setup-mysql file :

<br />

In order to install "MySQL" from the shell bash script (Linux operating system), run the following commands :<br />
1 - `sudo apt-get update`, this command updates the package information.<br />
2 - `sudo apt-get install mysql-server`, give your password, if necessary, and hit the "ENTER" key. Press "y" to continue<br />
It will take some time to download and install "MySQL".
Now, it is time to set a password to secure mysql. Type the command :<br />
`sudo mysql_secure_installation`, press "y" to set the password, next choose from the options shown which one you de prefer. Create a password, then re-write it again and finally choose "y"s in what follows.<br />
After the installation and setup, you can login to MySQL using this command :<br />
`sudo mysql -u root`<br />
You can also show available databases by running this command :<br />
`show databases;`<br />
Now that mysql is ready, so as to import the database from the following backup file [Link to the backup file](https://www.mysqltutorial.org/wp-content/uploads/2018/03/mysqlsampledatabase.zip), you need to run these command first, your terminal. If you are already in mysql, typr `quit`, then:<br />
1 - `wget https://www.mysqltutorial.org/wp-content/uploads/2018/03/mysqlsampledatabase.zip`<br />
2 - `unzip mysqlsampledatabase.zip`<br />
Now go to mysql by typing `sudo mysql -u root`, then type the following :<br />
1 - `source /home/username/mysqlsampledatabase.sql`, this imports the database from the path in which it was saved.<br />
2 - `show databases;`, shows the databases available. NOTE: it can be typed both in lowercase and uppercase.<br />
3 - `CREATE USER 'username'@'localhost' IDENTIFIED BY '1234';`, this command line creates a user named "username".<br />
4 - `GRANT ALL PRIVILEGES ON * . * TO 'username'@'localhost';`, this grants to the user "username" all privileges. You can also use 'root' instead to have all privileges without typing this command.<br />

Now you are all caught up and ready to go.