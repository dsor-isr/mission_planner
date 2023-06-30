#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
append_file_name="$DIR/hosts_append_file"
BACKUP_IFS=$IFS
IFS=''
found_line=false
hosts_string=""
append_file="$(<$append_file_name)"
while IFS='' read -r line || [[ -n "$line" ]]; do
    if [ "$line" = "## DON'T CHANGE ANYTHING AFTER THIS LINE" ]; then
      found_line=true
      # echo "Found the line"
      break
    fi
    if [ "$found_line" = false ]; then
      #echo $line
      hosts_string="$hosts_string$line\n"
      #echo "[$hosts_string]"
    fi
done < /etc/hosts
hosts_string="$hosts_string$append_file"
# echo "-------------------"
# echo $append_file
# echo "-------------------"
echo -e $hosts_string > /etc/hosts
IFS=$BACKUP_IFS

