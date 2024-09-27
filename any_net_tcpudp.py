import select
import socket
import json
import datetime
import os
import logging
import sys

HOST = '192.168.8.2'
PORT = 48885
FASTLOADDIR = 'o:\\СОДК\\Сервер\\sodk\\'
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

inputstring = '{"id":"cam2","num":10,"dt":"2024-09-25 15:14:34","RSSI":-93,"NBbatt":3.316,"adclight": 620,"adcwater": 750,"adcwater2":   0,"cputemp":26.5,"temp":25.8,"humidity":36.5,"pressure":999.349,"flags":"0x00"}'
# dataname = ['dt', 'num', 'RSSI', 'Battery', 'Light', 'Water','Water2', 'CPUTemp', 'Temp', 'Humidity', 'Pressure', 'Flags']
# datanametag = ['RSSI', 'Battery', 'Light', 'Water', 'Water2','CPUTemp', 'Temp', 'Humidity', 'Pressure', 'Flags']


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
                    csvfile.write(key + ";")
            csvfile.write("\n")

        pass

    try:
        with open(csvFilename, 'a', newline='') as csvfile:
            csvfile.write("{};{};".format(js["dt"], js["num"]))
            for key in js:
                if (key != "id" and key != "num" and key != "dt"):
                    csvfile.write(str(js[key]).replace('.', ',') + ";")
            csvfile.write("\n")

    except Exception as e:
        logging.error(e)
        pass


def get_name(id):
    idn = 0
    if (id.startswith("cam")):
        idn = int(id[3:])
    else:
        return

    # Получение полного пути к файлу скрипта
    file_path = os.path.realpath(__file__)
    # Получение директории, в которой находится файл скрипта
    script_dir = os.path.dirname(file_path)
    file = script_dir + "/" + "names.txt"

    # open the file in read mode

    with open(file, 'r') as file:
        # read lines from the file
        lines = file.readlines()

    res = {}

    for line in lines:
        # read lines from the file
        key, value = line.strip().split(':')
        # print("{}: {}".format(key, value))
        res[key.strip()] = value.strip()

    return res[str(idn)]


# create Wonderware Historian Fast load
def write_historian(js):
    id = js["id"]
    name = get_name(id)

    IntouchFilename = '{}{} {:%Y-%m-%d_%H.%M.%S.%f}.csv'.format(
        FASTLOADDIR, id, datetime.datetime.now())
    try:
        with open(IntouchFilename, 'w', encoding="latin-1") as f:
            f.write("ASCII\n,\n")
            f.write("SODK,1,Server Local,1,1\n")
            dt = datetime.datetime.strptime(js["dt"], "%Y-%m-%d %H:%M:%S")
            time_and_date = '{:%Y/%m/%d,%H:%M:%S}.000'.format(dt)
            # for d in datanametag:
            #    print("compare {}".format(d))
            # print("compare {}: {}, {}\n".format(d, d in js, js[d] != ''))
            for key in js:
                if (key != "id" and key != "num" and key != "dt"):
                    f.write("TK{}_{},0,{},0,{},192\n".format(name.encode(
                        encoding="latin-1", errors="ignore").decode(), key, time_and_date, js[key]))

    except Exception as e:
        # print('File error! {}'.format(e))
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

    write_historian(json.loads(inputstring))

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
                        if "id" in js:
                            write_csv(js)
                            write_historian(js)

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
