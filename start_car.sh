#! /bin/bash

echo -e " \e[1m\e[34m _    ________________________  ____ \e[97m             _________ \e[21m";
echo -e " \e[1m\e[34m| |  / / ____/ ____/_  __/ __ \/ __ \ \e[97m           /__  / __ \ \e[21m";
echo -e " \e[1m\e[34m| | / / __/ / /     / / / / / / /_/ / \e[97m ______      / / /_/ /\e[21m";
echo -e " \e[1m\e[34m| |/ / /___/ /___  / / / /_/ / _, _/  \e[97m/_____/     / /\__, / \e[21m";
echo -e " \e[1m\e[34m|___/_____/\____/ /_/  \____/_/ |_|   \e[97m           /_//____/  \e[21m";
echo -e "                                                           ";
echo -e "            \e[1m\e[33m    ____  ___   ___________   ________\e[21m";
echo -e "            \e[1m\e[33m   / __ \/   | / ____/  _/ | / / ____/\e[21m";
echo -e "            \e[1m\e[33m  / /_/ / /| |/ /    / //  |/ / / __  \e[21m";
echo -e "            \e[1m\e[33m / _, _/ ___ / /____/ // /|  / /_/ /  \e[21m";
echo -e "            \e[1m\e[33m/_/ |_/_/  |_\____/___/_/ |_/\____/   \e[21m";
echo "                                      ";

roslaunch vector79-jetson.launch & ssh pi@vector79-pi.local "bash -ic  'roslaunch code/ARCRacing/vector79-pi.launch'" & rosservice call /camera/start_capture

