import serial
from threading import Thread
from functools import wraps
from time import sleep

def thrd(f):
    ''' This decorator executes a function in a Thread'''
    @wraps(f)
    def wrapper(*args, **kwargs):
        thr = Thread(target=f, args=args, kwargs=kwargs)
        thr.start()
        
    return wrapper

class IGUS:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout       
        self.on_move = False
        self.pattern_flag = False
        
        self.connect()  
        
    def connect(self):
        self.ser = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
        self.read_msg()
        self.read_msg()
    
    def reconnect(self):
        self.ser.close()
        self.connect()
        
    def close(self):
        self.ser.close()
    
    def read_msg(self):
        msg=""
        while True:
            line = self.ser.readline().decode('utf-8')
            if line:
                msg += line                        
            else:
                break                   
        if msg:
            print(msg)
            return msg
            
    def send_msg(self, msg):
        if msg != 'S':
            while self.on_move:
                pass
        self.ser.write(str.encode(msg))
        
    def display_help(self):
        self.send_msg('h')
        self.read_msg()
        
    def move_stop(self):
        self.send_msg('S')
        self.read_msg()
        
    def move_home(self):
        self.send_msg('H')
        self.check_move(reach_msg='HOME POSITION\r\n') 
           
    def move_position(self, value, unit="mm", direction="R"):
        """_summary_
        Move linear module to
        Args:
            value (_type_): value to move
            unit (str, optional): can be mm or step. Defaults to "mm".
            direction (str, optional): can be R or L. Defaults to "R".
        """
        
        if unit == "mm":
            value = self.get_meters_value(value = value)
        print(f'{direction} -- {round(value*0.00375)} -- mm ')
        print(f'{direction} -- {value} -- step ')
        self.send_msg(f'{direction}#{value}')
        self.check_move()

    @thrd
    def check_move(self, reach_msg='POSITION REACHED\r\n'):
        self.on_move = True
        while self.read_msg() != reach_msg:
            sleep(0.5)
            pass
        self.on_move = False
        
    def get_meters_value(self, value):
        return round(value/0.00375)
    
    @thrd  
    def run_pattern(self, values, unit="mm", direction="R"):
        """_summary_

        Args:
            values (_type_): array of values to move
            unit (str, optional): _description_. Defaults to "mm".
            direction (str, optional): _description_. Defaults to "R".
        """
        for value in values:
            while (self.on_move or not self.pattern_flag):
                pass
            self.move_position(value=value, unit=unit, direction=direction) 
            self.pattern_flag = False

    def get_current_position(self, unit="mm"):
        self.send_msg('P')
        if unit == "mm":
            return float(self.get_meters_value(float(self.read_msg())))
        else : 
            return float(self.read_msg())
            
        
if __name__ == '__main__':
    values = [10, 20, 30, 35]
    igus = IGUS(port="/dev/ttyUSB2")
    igus.move_home()
    # igus.run_pattern(values, unit="mm", direction="R")
    # igus.display_help()
    
    
