	 / _____)             _              | |
	( (____  _____ ____ _| |_ _____  ____| |__
	 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
	 _____) ) ____| | | || |_| ____( (___| | | |
	(______/|_____)_|_|_| \__)_____)\____)_| |_|
	  (C)2019 Semtech

How to auto-start the with systemd
==================================

## Create a new systemd service

Update the lora_pkt_fwd.service file with proper paths and options, then

```console
sudo cp lora_pkt_fwd.service /etc/systemd/system
```

### Enable the service for autostart with

```console
sudo systemctl daemon-reload
sudo systemctl enable lora_pkt_fwd.service
sudo reboot
```

### The following commands to disable the service, manually start/stop it:

```console
sudo systemctl disable lora_pkt_fwd.service
```

```console
sudo systemctl start lora_pkt_fwd.service
```

```console
sudo systemctl stop lora_pkt_fwd.service
```

## Configure rsyslog to redirect the packet forwarder logs into a dedicated file

```console
sudo cp lora_pkt_fwd.conf /etc/rsyslog.d
sudo systemctl restart rsyslog
```

### See the logs

```console
sudo journalctl -u lora_pkt_fwd -f
```

or

```console
cat /var/log/lora_pkt_fwd.log
```