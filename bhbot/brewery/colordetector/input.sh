if ! [ -e input ]; then
    mkfifo input
fi

while true
do
    read line
    echo $line > input
done
