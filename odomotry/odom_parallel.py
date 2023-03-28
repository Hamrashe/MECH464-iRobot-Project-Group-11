import threading
import time

def odom():
    print('Odom is being read\n')



t = threading.Timer(5.0, odom)
t.start()

while True:
    print('odom is not being read\n')
    time.sleep(1)
