# creer regle pour dongle usb activé en sortie par defaut

# lister les cartes son
$ aplay -l
--> card 0: Device [USB Audio Device], device 0: USB Audio [USB Audio]

# test sortie son
$ speaker-test -D hw:0,0 -c 2

# trouver les infos de la carte
$ lsusb
--> Bus 001 Device 004: ID 0d8c:0014 C-Media USB Audio Device

--------------------
Vendor = 0d8c
Product = 0014
carte USB = C-Media
--------------------

# creer regle udev
$ sudo nano /etc/udev/rules.d/90-usb-audio.rules
SUBSYSTEM=="sound", ATTRS{idVendor}=="0d8c", ATTRS{idProduct}=="0014", SYMLINK+="snd/usb_audio"

# activer
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger

# blacklister l'audio hdmi de la orin nx
$ sudo nano /etc/modprobe.d/blacklist-tegra-hdmi-audio.conf
blacklist snd_hda_tegra
blacklist snd_hda_codec_hdmi

$ sudo update-initramfs -u

$ sudo nano /boot/extlinux/extlinux.conf
apres APPEND, ajouter : modprobe.blacklist=snd_hda_tegra,snd_hda_codec_hdmi

------------------------------------------------------------------------------
nvidia@ubuntu:~$ lsmod | grep hda
nvidia@ubuntu:~$ aplay -l
**** List of PLAYBACK Hardware Devices ****
card 0: Device [USB Audio Device], device 0: USB Audio [USB Audio]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
------------------------------------------------------------------------------

sudo apt update
sudo apt install libportaudio2 libportaudiocpp0 portaudio19-dev
pip3 install sounddevice
pip install piper-tts

-----------------

ros2 launch qbo_tts qbo_tts_launch.py
ou
ros2 run qbo_tts qbo_tts_node (pour voir les logs /temps)