#!/bin/sh

appdemo=appDemo
cpss_init_system_cmd="cpssInitSystem 35,7,0 portMgr" # Falcon-4t
ethdrv=yuval/eth-stuff/mvEthDrv.ko

mg=0
sdma_rx_queue=0
sdma_tx_queue=5
subnet_mask=24
port_netdev_mac_prefix="00:50:43"
port_netdev_ip_suffix="19.1.10"
tx_dsa_tags_file=/tmp/tx_dsa_tags # list of tx_tsa tags, line per port 16 bytes hex format, no 0x prefix
port_rx_dsa_mask="00 F8 00 00 00 00 0c 00 00 30 00 00 00 00 00 00"

random_mac_file=/tmp/random_mac_file
cpss_cmd_file=/tmp/cpss_cmd_file.lua
cpss_netdev_demo_cmd="appDemoNetDevDemo"

# Some validations
if [ ! -f ${tx_dsa_tags_file} ]; then
	echo "File ${tx_dsa_tags_file} not found"
	exit 1
fi
if [ ! -f ${ethdrv} ]; then
	echo "File ${ethdrv} not found"
	exit 1
fi
if [ ! -f ${appdemo} ]; then
	echo "File ${appdemo} not found"
	exit 1
fi

# How many ports are configure
t1=$(wc -l ${tx_dsa_tags_file})
set ${t1}
max_ports=$1
if [ ${max_ports} -lt 1 ]; then
	echo "File ${tx_dsa_tags_file} is empty"
	exit 1
fi

# Create config file to be run when appDemo starts if not exist
# Set the last parameter ignorePortCfg=0 while calling appDemoNetDevDemo()
# Vaule 0 means the multi-port port configuration is applied with the
# default setting (VLAN, PVID, MAC entry).
# Value 1 means the application apply the port configuration manually.
if [ ! -f ${cpss_cmd_file} ]; then
	echo "${cpss_init_system_cmd}" > ${cpss_cmd_file}
	echo "do shell-execute ${cpss_netdev_demo_cmd} \"${random_mac_file}\" \"${port_netdev_mac_prefix}\" ${sdma_rx_queue} 0" >> ${cpss_cmd_file}
fi

rm -f ${random_mac_file}

echo -n "Executing ${appdemo} "
# Run appDemo in background and wait for MAC file to be created
${appdemo} -daemon -config ${cpss_cmd_file}
while [ ! -f ${random_mac_file} ]; do sleep 1;echo -n "."; done
echo ""
read random_mac_byte_4_and_5 < ${random_mac_file}

echo "Loading ${ethdrv}"
# Load and configure eth driver
netdev_name="mvpp0"
/sbin/rmmod mvEthDrv 2>/dev/null
/sbin/insmod ${ethdrv}
# Let driver listen to all queues, if needed configure to which to listen
#sdma_rx_queue_mask=$(printf "0x%x\n" $((1 << ${sdma_rx_queue})))
sdma_rx_queue_mask=0xFF
echo ${mg} > /sys/class/net/${netdev_name}/mg
echo ${sdma_rx_queue_mask} > /sys/class/net/${netdev_name}/rx_queues
echo ${sdma_tx_queue} > /sys/class/net/${netdev_name}/tx_queue
ip link set ${netdev_name} up arp on
# Create netdevs for all ports
#port_rx_dsa_mask=$(echo ${port_rx_dsa_mask} | sed 's/ / 0x/g' | sed 's/^/0x/')
echo "Creating ${max_ports} netdevs"
port_num=1
while [ ${port_num} -le ${max_ports} ];
do
	port_name=${netdev_name}.${port_num}
	port_tx_dsa_tag=$(sed -n ${port_num}p ${tx_dsa_tags_file})

	echo "${port_name} ${port_num}" > /sys/class/net/${netdev_name}/if_create
	port_num_hex=$(printf "%.2x" $((${port_num} - 1)))
	echo "${port_netdev_mac_prefix}:${random_mac_byte_4_and_5}:${port_num_hex}" > /sys/class/net/${port_name}/mac

	echo "${port_tx_dsa_tag}" > /sys/class/net/${port_name}/dsa
	echo "${port_rx_dsa_mask}" > /sys/class/net/${port_name}/rx_dsa_mask

	pn=$((${port_num} - 1))
	b1=$(($((${pn} & 0x1f)) << 3))
	b6=$(($(($((${pn} & 0x60)) >> 5)) << 2))
	b9=$(($(($((${pn} & 0x180)) >> 7)) << 4))
	port_rx_dsa_val=$(printf "00 %.2x 00 00 00 00 %02x 00 00 %.2x 00 00 00 00 00 00" ${b1} ${b6} ${b9})
	echo "${port_rx_dsa_val}" > /sys/class/net/${port_name}/rx_dsa_val

	ip addr add ${port_num}.${port_netdev_ip_suffix}/${subnet_mask} dev ${port_name}
	ip link set ${port_name} up arp on

	ifconfig ${port_name} up

	port_num=$((${port_num} + 1))
done
echo "Done"
exit 0
