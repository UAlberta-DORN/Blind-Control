import serial
import numpy as np
import time
import json
import requests
import csv
import datetime


# Serial Parameters
MOTOR_PORT = '/dev/ttyUSB0'
HUB_PORT = '/dev/ttyUSB1'
HUB_COMMAND = [{'command': 'Callback', 'id': ["7C:9E:BD:F4:27:A1", "AC:67:B2:0D:5F:81", "7C:9E:BD:F2:A6:D5"]}]
#HUB_COMMAND = '[{\'command\':\'Callback\',\'id\':["7C:9E:BD:F4:27:A1", "3C:61:05:3D:D3:79", "AC:67:B2:0D:5F:81", "7C:9E:BD:F2:A6:D5"]}]'.encode('utf-8')
BAUD_RATE = 115200
NUM_SATELLITES = 3


# Motor Parameters
MAX_A_STEPS = 3800
MAX_B_STEPS = 1200
A_STEP_SIZE = 0.25
B_STEP_SIZE = 0.8
ERROR_THRESHOLD = 0.01
LIGHT_TIMEOUT = 5


# Periods [s]
DATA_PERIOD = 5
API_PERIOD = 5
CONTROL_PERIOD = 10
LOG_PERIOD = 10

# Web App
BASE_PATH = 'http://127.0.0.1:80/'
TEMP_PATH = BASE_PATH + 'dorn/0'
LIGHT_PATH = BASE_PATH + 'dorn/1'
HEIGHT_PATH = BASE_PATH + 'dorn/10'
TILT_PATH = BASE_PATH + 'dorn/11'
ALL_PATH = BASE_PATH + 'dorn/all'
CSV_PATH = r'graphData.csv'
CSV_ROW = ['2021-02-10', '12:00', '20', '20', '20', '20', 'False', 'False', 'False', '20', '20', 'False', '20', '20']


def percent_2_lux(percent):
    return (percent ** 2) / 100


def lux_2_percent(lux):
    lux = np.clip(lux, 0, 1000)
    return round(np.sqrt(10 * lux), 1)

def percent_2_a(percent):
    return int(MAX_A_STEPS * a)

def a_2_percent(a):
    return round(percent / MAX_A_STEPS, 3)


def percent_2_b(percent):
    return -int(2 * MAX_B_STEPS * percent / np.pi)


def b_2_percent(b):
    return round(-np.pi * b / (2 * MAX_B_STEPS), 3)


def interpret_hub_data(dictionary):
    temp = 0
    light = 0
    n = 0
    for child in dictionary['children']:
        temp += dictionary['children'][child]['data']['temp']
        light += dictionary['children'][child]['data']['light']
        n += 1
    return temp / n, light / n


class Main_Controller:
    def __init__(self):
        # Communication with motors
        self.motor_com = serial.Serial(MOTOR_PORT, BAUD_RATE)
        self.motor_com.flush()
        self.motor_is_running = False

        # Communication with hub
        self.hub_com = Hub_Serial(HUB_PORT, baud_rate=BAUD_RATE)
        try:
            self.hub_com.close_connection()
        except:
            pass
        self.hub_com.connect_serial()

        # Light and Temp Values
        self.temp = 0
        self.light = 0

        # Motor parameters
        self.up_pos = 1
        self.tilt_pos = np.pi / 180
        
        self.update_api()

    def set_blind_pos(self, up_pos, tilt_pos):
        self.up_pos = up_pos
        self.tilt_pos = tilt_pos

    def update_data(self):
        self.hub_com.send_object(HUB_COMMAND)
        time.sleep(0.1)
        self.hub_com.ser.flushInput()
        self.hub_com.get_message()
        try:
            self.temp, self.light = interpret_hub_data(self.hub_com.message)
        except:
            pass
    
    def update_api(self):
        temp = json.dumps({'value': round(self.temp, 1)})
        light = json.dumps({'value': lux_2_percent(self.light)})
        height = json.dumps({'value': 100 * self.up_pos})
        tilt = json.dumps({'value': round(200 * self.tilt_pos / np.pi, 1)})
        requests.put(TEMP_PATH, json=temp)
        requests.put(LIGHT_PATH, json=light)
        requests.put(HEIGHT_PATH, json=height)
        requests.put(TILT_PATH, json=tilt)
        data = requests.get(ALL_PATH).json()
        self.manual_blinds = bool(data[6]['value'])
        
    def control(self):
        data = requests.get(ALL_PATH).json()
        ref_light = int(data[3]['value'])
        for i in range(LIGHT_TIMEOUT):
            self.update_data()
            self.update_api()
            err = np.clip((ref_light - lux_2_percent(self.light)) / 100, -1, 1)
            if abs(err) <= ERROR_THRESHOLD or self.light == 0.0:
                break
            dh = -A_STEP_SIZE * err * np.cos(self.tilt_pos)
            dh = np.clip(self.up_pos + dh, 0, 1) - self.up_pos
            dtheta = B_STEP_SIZE * err * self.up_pos * np.sin(self.tilt_pos)
            dtheta = np.clip(self.tilt_pos + dtheta, np.pi / 180, np.pi / 2) - self.tilt_pos
            a_steps = percent_2_a(dh)
            b_steps = percent_2_b(b_steps)
            self.motor_com.flush()
            self.motor_com.write((bytes('A{}'.format(a_steps), 'utf-8')))
            print('A{}'.format(a_steps))
            time.sleep(0.1)
            while True:
                if self.motor_com.inWaiting() > 0:
                    message = self.motor_com.readline().decode('utf-8').rstrip()
                    print(message)
                    if message == 'P':
                        break
            self.up_pos += a_2_percent(a_steps)
            self.motor_com.flush()
            self.motor_com.write((bytes('B{}'.format(b_steps), 'utf-8')))
            while True:
                if self.motor_com.inWaiting() > 0:
                    message = self.motor_com.readline().decode('utf-8').rstrip()
                    print(message)
                    if message == 'Q':
                        break
            self.tilt_pos += b_2_percent(b_steps)
            
    def manual_control(self):
        data = requests.get(ALL_PATH).json()
        ref_height = int(data[7]['value']) / 100
        ref_tilt = np.pi * int(data[8]['value']) / 200
        a_steps = percent_2_a(ref_height - self.up_pos)
        b_steps = percent_2_b(ref_tilt - self.tilt_pos)
        self.motor_com.flush()
        self.motor_com.write((bytes('A{}'.format(a_steps), 'utf-8')))
        print('A{}'.format(a_steps))
        time.sleep(0.1)
        while True:
            if self.motor_com.inWaiting() > 0:
                message = self.motor_com.readline().decode('utf-8').rstrip()
                print(message)
                if message == 'P':
                    break
        self.up_pos += a_2_percent(a_steps)
        self.motor_com.flush()
        self.motor_com.write((bytes('B{}'.format(b_steps), 'utf-8')))
        while True:
            if self.motor_com.inWaiting() > 0:
                message = self.motor_com.readline().decode('utf-8').rstrip()
                print(message)
                if message == 'Q':
                    break
        self.tilt_pos += b_2_percent(b_steps)
        
    def log_data(self):
        csv_row = CSV_ROW.copy()
        csv_row[0] = datetime.date.today().strftime("%Y-%m-%d")
        csv_row[1] = datetime.now().strftime("%H:%M:%S")
        data = requests.get(ALL_PATH).json()
        for i in range(2, 12):
            csv_row[i] = data[i]['value']

        #Add row to csv file
        with open(CSV_PATH, 'a+', encoding='utf-8', newline='') as csvf:
            writer = csv.writer(csvf)
            writer.writerow(csv_row)
        
                        
    def main_loop(self):
        data_time = time.time()
        api_time = time.time()
        control_time = time.time()
        log_time = time.time()
        print('Start')
        while True:
            if (time.time() - data_time) >= DATA_PERIOD:
                data_time = time.time()
                self.update_data()
            if (time.time() - api_time) >= API_PERIOD:
                api_time = time.time()
                self.update_api()
            if (time.time() - control_time) >= CONTROL_PERIOD:
                control_time = time.time()
                if self.manual_blinds:
                    self.manual_control()
                else:
                    self.control()
            if (time.time() - log_time) >= LOG_PERIOD:
                log_data()


class Hub_Serial:
    def __init__(self, comm_port, baud_rate=115200, timeout=10, patience=5):
        self.comm_port = comm_port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.patience = patience
        self.message = {}

    def connect_serial(self, patience=None):
        if patience == None:
            patience = self.patience
        if patience >= 0:
            try:
                self.ser = serial.Serial(self.comm_port, self.baud_rate, timeout=self.timeout)
                time.sleep(0.03)
                self.ser.flushInput()
                self.ser.flushOutput()
                time.sleep(0.03)
            except:
                try:
                    self.ser.close()
                except:
                    pass
                self.connect_serial(patience=patience - 1)
        else:
            print("Error: cannot connect to the Hub\n")

    def reset_device(self):
        self.ser.flushInput()
        self.ser.flushOutput()
        time.sleep(0.03)
        self.ser.setDTR(False)
        time.sleep(0.03)
        self.ser.setDTR(True)
        time.sleep(0.03)

    def close_connection(self):
        self.ser.close()

    def get_message(self, patience=None):
        if patience == None:
            patience = int(self.patience)
        elif patience == 1:
            self.reset_device()
            self.get_message(patience=0)
        elif patience < 0:
            return
        # print(f'patience = {patience}\n')
        ser_bytes = self.ser.readline()
        decoded_bytes = (ser_bytes[0:len(ser_bytes) - 2].decode("ISO-8859-1"))
        try:
            self.message = json.loads(decoded_bytes)
        except:
            # self.get_message(patience = patience-1)
            pass

    def send_object(self, json_obj={}):
        if json_obj == {}:
            print("Please send something useful, thank you")
            return
        mesg_to_send = json.dumps(json_obj).encode('utf-8')
        self.ser.write(mesg_to_send)

        pass


if __name__ == '__main__':
    main_controller = Main_Controller()
    main_controller.main_loop()