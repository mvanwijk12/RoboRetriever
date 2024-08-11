
echo "Initialise GPIO"
raspi-gpio set 19 op
raspi-gpio set 24 op
raspi-gpio set 19 dl

echo "Set direction pin high"
raspi-gpio set 24 dh
sleep 2

echo "Start stepping..."

while :
do
	echo "Step pin drive high"
	raspi-gpio set 19 dh
	sleep 0.0001

	echo "Step pin drive low"
	raspi-gpio set 19 dl
	sleep 0.0001
done
