#!/usr/bin/bash

source ~/team_name.sh ; 
sudo service cron stop; 
rm -f ~/barc/Dator/db.sqlite3; 
rm -f ~/default.cfg ; 
python ~/barc/Dator/manage.py syncdb --noinput; 