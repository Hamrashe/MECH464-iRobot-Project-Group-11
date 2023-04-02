from threading import Timer
import time



class RepeatTimer(Timer):  
    def run(self):  
        while not self.finished.wait(self.interval):  
            self.function(*self.args,**self.kwargs)  
            print(' ')  
            #call odometry
        
            

def printer():
    print('test\n')

t = RepeatTimer(0.5, printer)
t.start()
time.sleep(10)
t.cancel()