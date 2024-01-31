import argparse
import subprocess
import time

argparser = argparse.ArgumentParser()
argparser.add_argument('-n', '--pepper_ip', default="", help='pepper ip address', type=str)
argparser.add_argument('-w', '--pepper_password', default="", help='pepper password', type=str)

args = argparser.parse_args()

# ssh nao@<naoip>
# qicli call ALAutonomousLife.setState disabled
# qicli call ALMotion.wakeUp

# sshpass -p 'isir302' ssh nao@192.168.1.158 qicli call ALAutonomousLife.setState disabled
# sshpass -p 'isir302' ssh nao@192.168.1.158 qicli call ALMotion.wakeUp

subprocess.call(["sshpass", "-p", args.pepper_password, "ssh", "nao@" + args.pepper_ip, "qicli", "call", "ALAutonomousLife.setState", "disabled"])
time.sleep(1)
subprocess.call(["sshpass", "-p", args.pepper_password, "ssh", "nao@" + args.pepper_ip, "qicli", "call", "ALMotion.wakeUp"])
