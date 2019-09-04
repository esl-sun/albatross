import time
from os import listdir
from os.path import isfile, join
import RPi.GPIO as GPIO


PWR_pin = 23
logfile = "log-1.txt"
log_number = 0
log_directory = "/home/"
logSize = 0
log_entry = ""

GPIO.setmode(GPIO.BCM)
GPIO.setup(PWR_pin, GPIO.OUT)

def setup_gsm():
    #powercycle  the GSM
    GPIO.output(PWR_pin, GPIO.HIGH)
    time.sleep(3)
    GPIO.output(PWR_pin, GPIO.LOW)
    time.sleep(3)
    GPIO.output(PWR_pin, GPIO.HIGH)
    time.sleep(5)

    # Setup to the correct settings:
    ser.write("AT+CMGF=1\r") # set to text mode
    time.sleep(3)
    ser.write('AT+CMGDA="DEL ALL"\r') # delete all SMS
    time.sleep(3)
    reply = ser.read(ser.inWaiting()) # Clean buf

def send_log(log):
    ser.write('AT+CMGS="+27718207024"\r')
    time.sleep(3)
    msg = str(log)
    ser.write(msg + chr(26))
    time.sleep(3)
    ser.write('AT+CMGDA="DEL ALL"\r') # delete all
    time.sleep(3)
    ser.read(ser.inWaiting()) # Clear buf

def get_last_log_entry(logFile):
    last_line = ''
    f = open(logFile,"r")
    lines = f.read().splitlines()

    if(not lines): # list is empty
        f.close()
        return last_line

    last_line = str(lines[-1])
    f.close()

    return last_line

def get_log_file(log_dir,old_log_num,log_file):

    list_of_logs= [f for f in listdir(log_dir) if isfile(join(log_dir, f))]
    new_log_num = len(list_of_logs)


    if(new_log_num > old_log_num):

        log_file = log_file.replace(str(old_log_num),str(new_log_num))
        return [log_file,new_log_num]

    elif(new_log_num == old_log_num):
        return [log_file,new_log_num]

    else:
        return[log_file,0]



while(True):

    time.sleep(2)
    [logfile,log_number] = get_log_file(log_directory,log_number,logfile)

    if(log_number > 0 ):

        log_entry = get_last_log_entry(logfile)

        if(len(log_entry)>1):
            # print("log_entry: ", log_entry)
            setup_gsm()
            time.sleep(4)
            send_log(log_entry)
            time.sleep(300) # send every 5min a Log entry
