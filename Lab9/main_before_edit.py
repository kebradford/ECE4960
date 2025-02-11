#!/usr/bin/env python3
import asyncio
from bleak import discover, BleakClient
import time
from constants import Descriptors, Commands, getCommandName
from ece4960robot import Robot
from settings import Settings
from struct import unpack, calcsize

global nextIndex
global doScan
global get_odom
doScan = 0;

async def getRobot():
    devices = await discover(device=Settings["adapter"], timeout=2)
    # for d in devices:
    #    print(d.name)
    p_robot = [d for d in devices if d.name == "MyRobot"]
    if (len(p_robot) > 0):
        #    print(p_robot[0].address)
        return p_robot[0]
    else:
        return None


async def robotTest(loop):

    # Handle is the TX characteristic UUID; does not change in our simple case.
    # Robot sends "enq" every 2 seconds to keep the connection "fresh"
    # Otherwise, it's a struct of the form:
    # bytes(type + length + data)
    # This struct shouldn't be more than 99 bytes long.
    def simpleHandler(handle, value):
        global time  # This is apparently needed.
        if (value == "enq".encode()):
            pass

        else:
            fmtstring = "BB"+str(len(value)-2)+"s"
            code, length, data = unpack(fmtstring, value)
            '''
            Python doesn't have a switch statement, nor easily compatible
            enum support. This might be the easiest way to handle commands.
            '''
            if (Settings["OutputRawData"]):
                print(
                    f"Code: {getCommandName(code)} Length: {length} Data: {data}")

            # Somewhat detach console output from Bluetooth handling.
            if (code == Commands.SER_TX.value):
                theRobot.pushMessage(str(data, encoding="UTF-8"))

            # Example of unpacking a little-endian 32-bit float.
            if (code == Commands.GIVE_FLOAT.value):
                print(unpack("<f", data))

            # Example of command-response.
            if (code == Commands.PONG.value):
                print(f"Got pong: round trip {time.time() - theRobot.now}")
                if(Settings["pingLoop"]):
                    loop.create_task(theRobot.ping())
                    # theRobot.newPing = True

            # Unpack from an example stream that transmits a 2-byte and a
            # 4-byte integer as quickly as possible, both little-endian.
            global nextIndex, doScan, get_odom;
            
            if (code == Commands.BYTESTREAM_TX.value):
                #print("TODO: Handle test bytestream messages")
                
                if(unpack("<IQ", data)[0] >= nextIndex + 20):
                    print(unpack("<IQ", data))
                    print("next")
                    f = open("scan.txt", "a")
                    f.write(str(unpack("<IQ", data)))
                    f.write("\n")
                    f.close()
                    
                    nextIndex +=20
                
		

    async def checkMessages():
        while True:
            if(theRobot.availMessage()):
                print(f"BTDebug: {theRobot.getMessage()}")
            await asyncio.sleep(0.1)

    # You can put a UUID (MacOS) or MAC address (Windows and Linux)
    # in Settings["Cached"].
    if (not Settings["cached"]):
        theRobot_bt = await getRobot()

    else:
        theRobot_bt = type("", (), {})()
        theRobot_bt.address = Settings["cached"]
    # print(theRobot_bt.address)
    while (not theRobot_bt):
        print("Robot not found")
        theRobot_bt = await getRobot()

    if (not Settings["cached"]):
        print(f"New robot found. Must cache \
            {theRobot_bt.address} manually in settings.py")

    async with BleakClient(theRobot_bt.address, loop=loop, device=Settings["adapter"]) as client:
        # if (await client.is_connected()):
        #    print("Robot connected!")
        # srv = await client.get_services()
        # print(srv)
        await client.is_connected()
        theRobot = Robot(client, bleak=True)
        await client.start_notify(Descriptors["TX_CHAR_UUID"].value,
                                  simpleHandler)

        # await client.write_gatt_char(Descriptors["RX_CHAR_UUID"].value, msg)

        #async def myRobotTasks():
        def myRobotTasks():
            global doScan 
            commandFile = open("commandFile.txt", "r")
            command_read = commandFile.read()
            commandFile.close()
            print(command_read)
            if(str(command_read).strip() == "doScan"):
                doScan = 1;
                print("performing scan")

            #await theRobot.sendCommand(Commands.START_BYTESTREAM_TX)
         #   checkState = 1
          #  while(checkState == 1):
           #     await theRobot.sendCommand(Commands.START_BYTESTREAM_TX)
            #    checkState = 0
            #await theRobot.sendCommand(Commands.STOP_BYTESTREAM_TX)
            global getOdom
            getOdom = 1;
            command = input("Enter Command:") 
            print(command)

            if(doScan):
                 f = open("scan.txt", "w")
                 f.write("")
                 f.close()
                 print("Sending observe message")
                 global nextIndex 
                 nextIndex = -20;
                 await theRobot.sendMessage("observe")
                 await asyncio.sleep(1)
                 
                 doScan = 0;
                 
            
            if(getOdom):
                 f2 = open("odom.txt", "w")
                 f2.write("")
                 f2.close()
                 print("Sending odom message")
               
                 await theRobot.sendMessage("odom")
                 await asyncio.sleep(1)
                 
                 getOdom = 0;
                 await theRobot.sendCommand(Commands.START_BYTESTREAM_TX)
            await theRobot.sendCommand(Commands.START_BYTESTREAM_TX)

            #await theRobot.sendCommand(Commands.SER_RX)

            #pass

            #await theRobot.ping()

            #await theRobot.sendCommand(Commands.REQ_FLOAT)

            

            #await theRobot.testByteStream(5)
	


        async def motorLoop():
            while True:
                await theRobot.loopTask()
                await asyncio.sleep(0.1)

        await asyncio.gather(checkMessages(), myRobotTasks())
        # async for msg in checkMessages():
        #    print(f"BTDebug: {msg}")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(robotTest(loop))
    # theRobot = loop.run_until_complete(getRobot())
    # print(theRobot.address)