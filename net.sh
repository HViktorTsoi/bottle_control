if [ ! $1 ]; then
  echo "Please input the interface ID that LCM use (e.g. eth0)!"
else
  sudo ifconfig $1 multicast
  sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev $1
  export LCM_DEFAULT_URL="udpm://224.0.0.1:7667?ttl=1"
  echo "Done."
fi

