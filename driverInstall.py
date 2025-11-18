#!/usr/bin/env python3
import os
import subprocess
import re
import argparse

def GetDrivers(vrpathreg):
    proc = subprocess.Popen([vrpathreg], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, encoding='utf-8')
    (stdout, _) = proc.communicate()

    lineno = 0
    externalReg = re.compile('.*External Drivers:.*')
    lines = stdout.split('\n')
    for line in lines:
        lineno += 1
        if externalReg.match(line):
            break

    lines = [ x.strip().split(':')[1].strip() for x in lines[lineno:] if len(x) > 0]
    return lines

def RemoveDriver(vrpathreg, driver):
    cmd = [vrpathreg, 'removedriver', driver]
    print(cmd)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, encoding='utf-8')
    (_, _) = proc.communicate()

def AddDriver(vrpathreg, driver):
    cmd = [vrpathreg, 'adddriver', driver]
    print(cmd)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, encoding='utf-8')
    (_, _) = proc.communicate()

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('--toInstall', help="Driver to install (folder path that contains driver.vrdrivermanifest)", default=".")
    parser.add_argument('--vrpathreg', help="Path to vrpathreg.sh", default="/home/%s/.local/share/Steam/steamapps/common/SteamVR/bin/vrpathreg.sh" % os.getenv("USER"))

    args = parser.parse_args()

    fail = False
    if not os.path.exists(args.toInstall):
        print("Driver path [{args.toInstall}] does not exist")
        fail = True

    if not os.path.exists(args.vrpathreg):
        print("Path to vrpathreg.sh [{args.toInstall}] does not exist")
        fail = True

    if fail: 
        return

    vrpathreg = os.path.abspath(args.vrpathreg)
    to_install = os.path.abspath(args.toInstall)
    driver_name = os.path.basename(to_install)

    drivers = GetDrivers(vrpathreg)

    for driver in drivers:
        RemoveDriver(vrpathreg, driver)

    AddDriver(vrpathreg, to_install)

    for driver in drivers[::-1]:
        if driver_name in driver:
            continue

        AddDriver(vrpathreg, driver)

    pass

if __name__ == "__main__":
    main()

