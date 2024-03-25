#!/bin/bash
rm transform_output.txt
tail -f /var/log/syslog | grep --line-buffered "capture" > /home/suraj/RTES/RTES-Exercise4/Solution_zip/5/transform_output.txt