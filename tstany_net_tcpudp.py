import select
import socket
import json
import datetime
import os
import logging
import sys

HOST = '10.179.40.20'
PORT = 48885
# FASTLOADDIR = 'c:\\InSQL\\Data\\DataImport\\'
DATASIZE = 1024


def get_non_blocking_server_socket(type):
    # Создаем сокет, который работает без блокирования основного потока
    server = socket.socket(socket.AF_INET, type)
    server.setblocking(False)

    # Биндим сервер на нужный адрес и порт
    server.bind((HOST, PORT))

    if (type == socket.SOCK_DGRAM):
        return server

    # Установка максимального количество подключений
    server.listen(10)

    return server


def parse_msg(msg):
    st = 0
    fi = 0
    rez = []
    while (st >= 0 and fi >= 0):
        st = msg.find('{', fi)
        fi = msg.find('}', st)
        if (st >= 0 and fi > 0):
            rez.append(json.loads(msg[st:fi+1]))
    return rez


# create csv data file
# {"id":"12.1","num":1,"dt":"2000-12-12 00:00:58","U":542,"R":299999,"Ub1":9.764,"Ub0":12.035,"U0":0,"in":1,"T":28.7,"rssi":-63}

def write_csv(js):
    csvFilename = '{}_{:%Y-%m}.csv'.format(js["id"], datetime.datetime.now())
    try:
        with open(csvFilename, 'r', newline='') as csvfile:
            n = csvfile.read(1)
    except:
        with open(csvFilename, 'w', newline='') as csvfile:
            csvfile.write("dt;num;")
            for key in js:
                if (key != "id" and key != "num" and key != "dt"):
                    print(key)
                    if type(js[key]) is list:
                        if (len(js[key]) == 3):
                            csvfile.write("{}X;{}Y;{}Z;".format(key, key, key))
                    else:
                        csvfile.write("{};".format(key))

            csvfile.write("\n")
        pass

    try:
        with open(csvFilename, 'a', newline='') as csvfile:
            csvfile.write("{};{};".format(js["dt"], js["num"]))
            for key in js:
                if (key != "id" and key != "num" and key != "dt"):
                    s = js[key]
                    print(key, ":", s)
                    if type(s) is list:
                        if (len(s) == 3):
                            csvfile.write("{};{};{};".format(s[0], s[1], s[2]))
                    else:
                        csvfile.write("{};".format(s))
            csvfile.write("\n")

    except Exception as e:
        logging.error(e)
        pass


if __name__ == '__main__':

    str = '{"id":"cam1","num":1,"dt":"1970-01-01 00:00:00","RSSI":  0,"Battery":0.000,"Light":   0,"Water":1492,"WaterTemp":0.0,"Temp":27.6,"Humidity":36.0,"Pressure":0.000,"Acc":[-0.1,0.0,9.8],"Mag":[-20.5,5.6,-57.7],"Flags":"0x0200"}'
    j = json.loads(str)
    write_csv(j)
