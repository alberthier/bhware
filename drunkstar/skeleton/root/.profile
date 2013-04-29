export PATH=\
/bin:\
/sbin:\
/usr/local/bin:\
/usr/bin:\
/usr/sbin:\
/usr/bin/X11:

export PYTHONDONTWRITEBYTECODE=1

# If running interactively, then:
if [ "$PS1" ]; then

    export PS1='\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '

    export USER=`id -un`
    export LOGNAME=$USER
    export HOSTNAME=`/bin/hostname`
    export HISTSIZE=1000
    export HISTFILESIZE=1000
    export PAGER='/bin/less '
    export EDITOR='/usr/bin/vim'
    export INPUTRC=/etc/inputrc
    export LS_COLORS='no=00:fi=00:di=01;34:ln=01;36:pi=40;33:so=01;35:do=01;35:bd=40;33;01:cd=40;33;01:or=40;31;01:ex=01;32:*.tar=01;31:*.tgz=01;31:*.arj=01;31:*.taz=01;31:*.lzh=01;31:*.zip=01;31:*.z=01;31:*.Z=01;31:*.gz=01;31:*.bz2=01;31:*.deb=01;31:*.rpm=01;31:*.jar=01;31:*.jpg=01;35:*.jpeg=01;35:*.png=01;35:*.gif=01;35:*.bmp=01;35:*.pbm=01;35:*.pgm=01;35:*.ppm=01;35:*.tga=01;35:*.xbm=01;35:*.xpm=01;35:*.tif=01;35:*.tiff=01;35:*.mpg=01;35:*.mpeg=01;35:*.avi=01;35:*.fli=01;35:*.gl=01;35:*.dl=01;35:*.xcf=01;35:*.xwd=01;35:';

    alias ls='ls -F --color=auto'
    alias ll='ls -lh'
    alias la='ls -A'
    alias mv='mv -i'
    alias cp='cp -i'
    alias rm='rm -i'
    alias df='df -h'
    alias du='du -h'
    alias vi='vim'
    alias halt='busybox halt; exit'
    alias reboot='busybox reboot; exit'

    if [ -d "/root/bhware/bhbot/brewery" ]; then
        cd /root/bhware/bhbot/brewery
    fi
fi;
