after modifying or updating the module you have to run this:
```bash 
./dimos_rfid/integrate_with_dimos.sh
```

if you get a "permission error":
``` bash
chmod +x dimos_rfid/integrate_with_dimos.sh
# and then ./dimos_rfid/integrate_with_dimos.sh
```

Finally
``` bash
export ROBOT_IP=192.168.8.57
export RFID_API_BASE=http://10.42.200.240:8765/api/v1
dimos run unitree-go2-rfid
```