#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64
import asyncio
from bleak import BleakClient

async def main():
    pub = rospy.Publisher('force_sensor', Int64, queue_size=10)
    rospy.init_node('force_sensor', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    async with BleakClient("7C:9E:BD:3B:49:66" ) as client:
     read = await client.get_services()
     c = list(read.characteristics)
     #print(c)
     while not rospy.is_shutdown():
          out = await client.read_gatt_char(c[0])
          #print(out)
          #translate byte array to integer
          grams = int.from_bytes(out, "little")
          pub.publish(grams)
          rate.sleep()

    # Device will disconnect when block exits.

# Using asyncio.run() is important to ensure that device disconnects on
# KeyboardInterrupt or other unhandled exception.

     
if __name__ == '__main__':
   try:
        asyncio.run(main())
   except rospy.ROSInterruptException:
        pass