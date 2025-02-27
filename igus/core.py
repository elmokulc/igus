import serial
from threading import Thread
from functools import wraps
from time import sleep
import glob 
import sys
import logging


LOG_DEFAULT_FORMATTER = logging.Formatter(
    "[%(asctime)s:%(levelname)s:%(filename)s:%(lineno)d:%(funcName)s()]: [%(message)s]"
)

def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

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
        self.pass_check = False
        self.CONST_REV = 0.00375 # mm/step
        
        self.warning_words = [
        "ACK_WAIT"    ,
        "ACK_ERROR"      ,
        "ACK_INVALID"   ,
        "ACK_OUTRANGE"  ,
        "ACK_ABORTED"   ,
        "ACK_RESET"]

        self.setup_logger()        
        self.connect()  
    
    def setup_logger(self):
        # Configuration du logger
        self.logger = logging.getLogger(f"{IGUS.__name__}")
        handler = logging.StreamHandler()  # Sortie des logs sur la console
        handler.setFormatter(LOG_DEFAULT_FORMATTER)
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)  # Niveau par défaut
        self.logger.propagate = False  # Évite la duplication des logs
    
    def update_constant_rev(self, nb_step_per_tr=400, dist_per_tr=1.5):
        """_summary_

        Args:
            nb_step_per_tr (int, optional): Number of motor step to make one tour. Defaults to 400.
            dist_per_tr (float, optional): Displacement for one motor tour expressed in mm. Defaults to 1.5.
        """
        self.CONST_REV = dist_per_tr/nb_step_per_tr
    
        
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
            self.logger.info(f"received message: {msg}")
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
        self.check_cmd(reach_msg='HOME POSITION') 
           
    def move_position(self, value, unit="mm", direction="R"):
        """_summary_
        Move linear module to
        Args:
            value (_type_): value to move
            unit (str, optional): can be mm or step. Defaults to "mm".
            direction (str, optional): can be R or L. Defaults to "R".
        """
        
        if unit == "mm":
            value = value / self.CONST_REV
            self.logger.info(f'{direction} -- {round(value*self.CONST_REV)} -- mm ')
        else:
            self.logger.info(f'{direction} -- {value} -- step ')
        value = round(value)
        self.send_msg(f'{direction}#{value}')
        self.check_cmd(reach_msg='POSITION REACHED')

    @thrd
    def check_cmd(self, reach_msg='POSITION REACHED'):
        self.on_move = True
        while True:
            msg = self.read_msg()
            if self.pass_check:
                self.logger.info("Passing check")
                break
            if type(msg) != type(None):
                if reach_msg in msg:
                    break
                elif msg in self.warning_words:
                    self.logger.info(f"Warning: {msg}")
                    break
            self.logger.info("Waiting for confirmation...")
        self.on_move = False
        self.pass_check = False
        
    def get_meters_value(self, value):
        """_summary_
        Return step value in mm
        Args:
            value (_type_): _description_

        Returns:
            _type_: _description_
        """
        return value*self.CONST_REV
    
    
    
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
            
    def set_max_velocity(self, value):
        self.send_msg(f'VF#{value}')
        self.check_cmd(reach_msg='CMD DONE')


    def set_mode(self, mode="linear"):
        map_dic = {
            "linear":0,
            "rotate":1
        }        
        self.send_msg(f'M#{map_dic[mode]}')
        self.check_cmd(reach_msg='CMD DONE')

if __name__ == '__main__':
    # values = [10, 20, 30, 35]
    igus = IGUS(port="/dev/ttyUSB3")
    igus.set_mode("rotate")
    # igus.move_home()
    # igus.run_pattern(values, unit="mm", direction="R")
    # igus.display_help()
    # print(serial_ports())
    
    
