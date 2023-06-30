# Trials PC personal alias

# FAROL SSH connections

alias myellow='ssh medusa@myellow'          # Myellow
alias delfim='ssh delfim@delfim'              # Delfim
alias muned='ssh medusa@muned'              # Muned
alias mblack='ssh medusa@mblack'            # Mblack
alias mred='ssh medusa@mred'                # Mred
alias hrov_pts_otg='ssh oceantech@hrov_otg' # Hrov port side
alias mvector='ssh medusa@mvector'	        # Mvector

# *.* FAROL Safety Features
alias sf_myellow='python3 $(find ${ROS_WORKSPACE}/src/ -type d -iname watchdog)/src/WatchdogClient/watchdog_client_latency.py --vehicle myellow'
alias sf_mvector='python3 $(find ${ROS_WORKSPACE}/src/ -type d -iname watchdog)/src/WatchdogClient/watchdog_client_latency.py --vehicle mvector'
alias sf_mred='python3 $(find ${ROS_WORKSPACE}/src/ -type d -iname watchdog)/src/WatchdogClient/watchdog_client_latency.py --vehicle mred'
alias sf_mblack='python3 $(find ${ROS_WORKSPACE}/src/ -type d -iname watchdog)/src/WatchdogClient/watchdog_client_latency.py --vehicle mblack'
alias sf_muned='python3 $(find ${ROS_WORKSPACE}/src/ -type d -iname watchdog)/src/WatchdogClient/watchdog_client_latency.py --vehicle muned'
alias sf_delfim='python3 $(find ${ROS_WORKSPACE}/src/ -type d -iname watchdog)/src/WatchdogClient/watchdog_client_latency.py --vehicle delfim'
alias sf_glider='python3 $(find ${ROS_WORKSPACE}/src/ -type d -iname watchdog)/src/WatchdogClient/watchdog_client_latency.py --vehicle glider'

alias delfim_gen_enable='watch ssh delfim cansend can0 8006c000#A00F'
alias delfim_gen_disable='watch ssh delfim cansend can0 8006c000#D007'
