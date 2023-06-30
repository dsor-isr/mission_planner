```

Tinkers: DSOR Group
         malaclemys  -> jquintas@gmail.com

Descripton: Documentation of easy_alias
Date: 20200226

@~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~@
@         ?                                               @
@          __                                             @
@      ____||_                  ___                       @
@     (_______)            ,,  // \\                      @  
@      ____||_            (_,\/ \_/ \                     @
@     (_______)             \ \_/_\_/>                    @
@                            /_/  /_/         ______      @
@                                            |START |     @
@                                            |______|     @
@                                            |            @
@____________________________________________|____________@
```

# Easy_alias

Folder with handy alias and some bash scripts to beautify your terminal.

# Folder contents

| Folder | Folder Description |Contents |  
|:----------:|:----------|:---:|
| **glider_permanent_alias** |   Bunch of alias to make your life easier </br></br>   Also adds some scripts for mainly beautify your terminal located at the folder *glider_scripts_for_bash* </br></br>This are added when running the *config_glider_ros.sh* script | alias.sh |
| **glider_personal_allias** | personal alias that you want to add for better working with the vehicle. | *yourHostname_alias.sh* |
| **glider_scripts_for_bash**| autocompletes, terminal beautify |  autcomplete.sh </br></br>display_git_branch_in_prompt.sh</</br></br>git-completion.bash</br></br>roscat.sh</br></br>transfer.sh



# Important Notes

## How to make *glider_stack* alias available

You do this while installing the **glider_stack** in your machine, by running the following script:

```
sudo bash config_glider_ros_os.sh
```

or if you want to do manually add the following to your .bashrc:

```
export CATKIN_ROOT=<root of your ros folder> (often people use ~/)
export CATTER='pygmentize -g' 
source ${CATKIN_ROOT}/catkin_ws/src/glider_vx/glider_scripts/system_configurations/easy_alias/glider_permanent_alias/alias.sh
```
**Define well your CATKIN_ROOT, please**

And clone the glider_stack to the CATKIN_ROOT folder.

## How to add more alias to the default ones 

At the end of the *glider_permanent_alias/alias.sh* file you have the following lines:

```
##############################################################
# @.@ Personal configs if hostname file exists (vehicles included)
##############################################################
if [ -f "${CATKIN_ROOT}/catkin_ws/src/glider_vx/glider_scripts/system_configurations/easy_alias/glider_personal_alias/${HOSTNAME}_alias.sh" ]; then
	source ${CATKIN_ROOT}/catkin_ws/src/glider_vx/glider_scripts/system_configurations/easy_alias/glider_personal_alias/${HOSTNAME}_alias.sh
fi
```

This part basically checks if a file with your **hostname -> ${HOSTNAME}_alias.sh** exists at the folder *glider_personal_alias*. So if you want to add some personal alias do the following:

```
Imagine that your hostname is awesome

At the glider_personal_alias folder create the following file:
touch awesome_alias.sh

After this edit with your editor of choice and add as many alias as you want.
```

The content of the awesome_alias.sh can be something like this:
```
alias mvehicle1='ssh name@vehicleHostname1'  
alias mvehicle2='ssh name@vehicleHostname2'  
alias mvehicle3='ssh name@vehicleHostname3'  
alias mvehicle4='ssh name@vehicleHostname4'  
alias mvehicle5='ssh name@vehicleHostname5'  
```