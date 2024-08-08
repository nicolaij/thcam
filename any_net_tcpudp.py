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
    # "{\"id\"=\"cam1\",\"rssi\"=%d,\"NBbatt\"=%d,\"batt\"=%.2f,\"adclight\"=%.0f,\"adcwater\"=%.0f,\"adcwater2\"=%.0f,\"cputemp\"=%.1f,\"temp\"=%.1f,\"humidity\"=%.1f}"
    dataname = ['dt', 'num', 'temp', 'humidity', 'adclight', 'adcwater', 'adcwater2', 'cputemp', 'batt', 'NBbatt', 'rssi']
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

    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)-5.5s] %(message)s",
        handlers=[
            logging.FileHandler("debug.log"),
            logging.StreamHandler(sys.stdout)
        ]
    )

    # Откуда и куда записывать информацию
    outputs = []
    xinputs = []

    # Создаем серверный сокет без блокирования основного потока в ожидании подключения
    server_socket_tcp = get_non_blocking_server_socket(socket.SOCK_STREAM)
    server_socket_udp = get_non_blocking_server_socket(socket.SOCK_DGRAM)
    inputs = [server_socket_tcp, server_socket_udp]
    # print("TCP/UDP server is running")
    logging.info("TCP/UDP server is running")
    logging.debug("New: {}".format(str(server_socket_tcp)))
    logging.debug("New: {}".format(str(server_socket_udp)))

    while inputs:
        readables, writables, exceptional = select.select(
            inputs, outputs, xinputs)
        message = b''
        client_address = ('', 0)
        # print("readadle len: " + str(len(readables)))
        for s in readables:
            # print("protocol: " + str(s.proto))
            if s == server_socket_tcp:
                connection, client_address = s.accept()
                connection.setblocking(False)
                inputs.append(connection)
                logging.debug("New: {}".format(str(s)))
                continue
            else:  # UDP or connection
                # print("readadle: " + str(s))
                logging.debug("Msg: {}".format(str(s)))
                try:
                    (message, client_address) = s.recvfrom(DATASIZE)
                    # print("readadle: " + str(s.type))
                    if s.type == socket.SOCK_STREAM:
                        client_address = s.getpeername()
                except Exception as e:
                    # print(e)
                    logging.error(e)
                    pass

            if message:
                # Вывод полученных данных на консоль
                # print("{}: {}".format(str(client_address), str(message)))
                logging.info("{}: {}".format(
                    str(client_address), str(message)))
                try:
                    for js in parse_msg(message.decode(encoding="latin-1", errors="ignore")):
                        if js["id"]:
                            write_csv(js)
                except Exception as e:
                    # print(e)
                    logging.error(e)
                    pass
            else:
                # print("close: " + str(s))
                if s not in [server_socket_tcp, server_socket_udp]:
                    inputs.remove(s)
                logging.debug("Cls: {}".format(str(s)))
                s.close()
