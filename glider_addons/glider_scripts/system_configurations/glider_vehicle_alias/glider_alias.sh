#glider personal alias

##############################################################
# @.@ startup script - service
##############################################################
alias glider_start='sudo systemctl start gliderStartup.service'
alias glider_stop='sudo systemctl stop gliderStartup.service'
alias glider_restart='sudo systemctl stop gliderStartup.service && sudo systemctl start gliderStartup.service'


##############################################################
# @.@ bullet configuration
##############################################################
alias glider_bullet_internal='/usr/bin/parallel_port internal ssdoff'
alias glider_bullet_external='/usr/bin/parallel_port external ssdoff'


###############################################################
# @.@ View raw battery info
###############################################################
alias glider_bat_raw='rostopic echo /mvector0/drivers/bat_monit/raw/sentence'


######################
# @.@ ntpdate sync   #
######################

# glider ntpdate with console pc
alias glider_ntpdate='sudo ntpdate -bu ntpServer'

# ntpdate with the world
alias world_ntpdate='sudo service ntp stop && sudo ntpdate -s time.nist.gov && sudo service ntp start'

# Used before
bind 'set match-hidden-files off'
export LC_ALL=C
