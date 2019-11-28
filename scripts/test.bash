cd ../launch
gnome-terminal -e "roslaunch ./step1SITLUE4.launch"
echo "sleep 10s"
sleep 10
gnome-terminal -e "roslaunch ./step2.launch"
gnome-terminal -e "python ../scripts/simple_UDP.py"
