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

def write_csv():

    d = "{\"id\"=\"cam1\",\"rssi\"=23,\"NBbatt\"=343.4,\"batt\"=23,\"adclight\"=42,\"adcwater\"=24,\"adcwater2\"=64,\"cputemp\"=7,\"temp\"=999.6,\"humidity\"=33}"
    js = json.loads(d)
    dataname = ['temp', 'humidity', 'adclight', 'adcwater', 'adcwater2', 'cputemp', 'batt', 'NBbatt', 'rssi']
    for d in dataname:
        try:
            if js[d] == '':
                js[d] = ''
        except:
            js[d] = ''
            pass

    csvFilename = '{}_{:%Y-%m}.csv'.format(js["id"], datetime.datetime.now())
    try:
        with open(csvFilename, 'r', newline='') as csvfile:
            n = csvfile.read(1)
    except:
        with open(csvFilename, 'w', newline='') as csvfile:
            for x in dataname:
                csvfile.write(x + ";")

            csvfile.write("\n")
        pass

    try:
        with open(csvFilename, 'a', newline='') as csvfile:
            for x in dataname:
                csvfile.write(str(js[x]).replace('.', ',') + ";")

            csvfile.write("\n")
    except Exception as e:
        logging.error(e)
        pass


if __name__ == '__main__':

    write_csv()
