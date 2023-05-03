# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
[ -z "$PS1" ] && return

# don't put duplicate lines in the history. See bash(1) for more options
# ... or force ignoredups and ignorespace
HISTCONTROL=ignoredups:ignorespace

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "$debian_chroot" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto --time-style=long-iso'
    #alias dir='dir --color=auto --time-style=long-iso'
    #alias vdir='vdir --color=auto --time-style=long-iso'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if [ -f /etc/bash_completion ] && ! shopt -oq posix; then
    . /etc/bash_completion
fi



### ROS ##################################################

## ROS aliases

alias cdros='cd ~/catkin_ws'
alias cdross='cd ~/catkin_ws/src'
alias makeros='cd ~/catkin_ws && catkin_make ; echo ""; echo "build done"; cd -'

## load ROS base scripts

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash


## TurtleBot3 model definition

export TB3_MODEL=burger
export TURTLEBOT3_MODEL=${TB3_MODEL}


## ROS environments breifing
ros_env(){
	echo -e "\n\e[92mROS_HOSTNAME\e[0m(Current IP): $ROS_HOSTNAME"
	echo -e "\e[92mROS_MASTER_URI\e[0m: $ROS_MASTER_URI"
	echo -e "\e[92mTB3_MODEL\e[0m: $TB3_MODEL"
	echo -e "To check this again, type \e[92mros_env\e[0m."
	echo -e "To edit ~/.bashrc to change ROS_MASTER_URI, type \e[92meb\e[0m.\n"
}

## hostname definition

export ROS_HOSTNAME=$(hostname -I | cut -d' ' -f1)


## ROS master (roscore) address setup [ IMPORTANT 중요 ]
##    ROS_HOSTNAME should NOT be "localhost" --> cannot connect from/to other machine
##    ROS_HOSTNAME을 localhost로 하면 이 PC에서 실행하는 roscore에 다른 컴퓨터/노드가 접속할 수 없으므로 사용하지 말 것!
##    Make sure ROS_MASTER_URI should be set up correctly!
##    아래 줄 ROS_MASTER_URI의 IP를 roscore 실행하는 컴퓨터의 IP에 맞출 것!

# When roscore runs on this PC (recommended)
export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311

# When roscore runs on specific PC (Manual IP set)
#export ROS_MASTER_URI=http://192.168.0.201:11311



##########################################################

## .bashrc edit alias

EB_START_LINE=151
alias eb="nano +$EB_START_LINE ~/.bashrc ; source ~/.bashrc; echo '.bashrc updated and applied!'"
alias ebq="source ~/.bashrc; echo '~/.bashrc applied!'"


## apt update alias

alias au="sudo apt update; sudo apt -y -o Dpkg::Options::="--force-overwrite" upgrade; sudo apt autoremove -y; sudo apt clean"


## SSH Startup notification on Byobu

if [ -z "$_motd_listed" ]; then
	case "$TMUX_PANE" in
	%1)
		## MOTD by system
		cat /run/motd.dynamic

		## ROS information breif prompt on login
		ros_env

		export _motd_listed=yes
		;;
	*)
		;;
	esac
fi
