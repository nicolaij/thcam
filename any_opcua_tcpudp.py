import select
import socket
import json
import datetime
import os
import logging
import sys
import queue
from opcua import ua, Server
import pyodbc

HOST = '10.179.40.20'
PORT = 48885
FASTLOADDIR = 'o:\\СОДК\\Сервер\\sodk\\'
DATASIZE = 1024
ODBCSQLCONNECT = 'DSN=zuluSQL;UID=asutp;PWD=asutp'

OPCNODES_CAM = [1, 2, 3, 6]
OPCNODES_SODK = [9]


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
# {"id":"cam2","num":2,"dt":"2025-07-30 10:25:08","Battery":3.260,"RSSI":-77,"Light":78.8,"Water":0.0,"WaterTemp":31.7,"Temp":26.9,"Humidity":55.1,"Flags":"0x0010","Acc":[-0.51,-0.16,-1.10],"Mag":[-0.05,0.09,0.51],"tac":23,"ci":153625089}

# inputstring = {"id":"cam2","num":8,"dt":"2025-03-31 09:38:08","RSSI":-95,"Battery":3.268,"Light":9.9,"Water":0.0,"WaterTemp":0.0,"Temp":26.0,"Humidity":32.7,"Acc":[0.00,0.00,0.00],"Mag":[0.00,0.00,0.00],"Flags":"0x0680"}

# dataname = ['dt', 'num', 'RSSI', 'Battery', 'Light', 'Water','Water2', 'CPUTemp', 'Temp', 'Humidity', 'Pressure', 'Flags']
# datanametag = ['RSSI', 'Battery', 'Light', 'Water', 'Water2','CPUTemp', 'Temp', 'Humidity', 'Pressure', 'Flags']

cam_dataname = ['dt', 'num', 'Battery', 'Light', 'Water', 'WaterTemp', 'Temp', 'Humidity', 'Flags', 'AccX', 'AccY', 'AccZ', 'MagX', 'MagY', 'MagZ', 'TAC', 'CI', 'RSSI']
# {"id":"sodk9.5","num":1,"dt":"1970-01-01 00:00:01","U":12.0,"R":5999,"U0":11.8,"Ubatt1":12.494,"time":63,"Flags":"0x0010","NBbatt":3.365,"RSSI":-65,"Temp":31.7}
sodk_common_dataname = ['dt', 'num', 'Flags', 'Temp', 'TAC', 'CI', 'RSSI']
sodk_channel_dataname = ['dt', 'num', 'U', 'R', 'U0', 'Ubatt1', 'time', 'Flags']
sodk_dataname = sodk_channel_dataname + ['Temp', 'TAC', 'CI', 'RSSI']


def write_SQL(js):
    id = str(js["id"]).split('.')
    dataname = []
    sqlcolumns = []
    sqlvalues = []
    db = "SQL_ASUTP_SODKData"
    if id[0].startswith("sodk"):
        dataname = sodk_dataname
        sqlcolumns = ["pribor", "channel", "idf"]
        sqlvalues = [int(id[0][4:]), int(id[1]), float(str(js["id"])[4:])]
        try:
            val = int(str(js['Flags']), 16)
            sqlcolumns.append('door')
            sqlvalues.append(val & 1)
            sqlcolumns.append('loop')
            sqlvalues.append((val >> (int(id[1]) + 7)) & 1)
        except Exception as e:
            logging.error(e)

    elif id[0].startswith("cam"):
        db = "SQL_ASUTP_THCAMData"
        dataname = cam_dataname
        sqlcolumns = ["id"]
        sqlvalues = [str(js["id"])[3:]]
    else:
        return

    for key in js:
        if key == "Acc":
            sqlcolumns.extend(['AccX', 'AccY', 'AccZ'])
            sqlvalues.extend(js["Acc"])
        elif key == "Mag":
            sqlcolumns.extend(['MagX', 'MagY', 'MagZ'])
            sqlvalues.extend(js["Mag"])
        elif key == "dt":
            sqlcolumns.append(str(key))
            sqlvalues.append(datetime.datetime.strptime(js["dt"], "%Y-%m-%d %H:%M:%S"))
        elif key == "Flags":
            sqlcolumns.append(str(key))
            sqlvalues.append("'{}'".format(str(js[key])))
        elif key in dataname:
            sqlcolumns.append(str(key))
            sqlvalues.append(str(js[key]))

    # import pyodbc
    # conn = pyodbc.connect('DSN=zuluSQL2;UID=asutp;PWD=asutp')
    # cursor.execute("INSERT INTO SQL_ASUTP_THCAMData (idauto,dt) VALUES (1,'2025-03-31 09:38:08')")
    conn = pyodbc.connect(ODBCSQLCONNECT, ansi=True)
    cursor = conn.cursor()
    # print("INSERT INTO {} ({}) VALUES ({})".format(db, ", ".join(sqlcolumns), ", ".join("?" * len(sqlcolumns))))
    cursor.execute("INSERT INTO {} ({}) VALUES ({})".format(db, ", ".join(sqlcolumns), ", ".join("?" * len(sqlcolumns))), tuple(sqlvalues))
    # Подтверждаем внесение изменений
    conn.commit()
    # Закрываем соединение
    cursor.close()
    conn.close()


def write_csv(js):
    id = str(js["id"])

    csvFilename = '{}_{:%Y-%m}.csv'.format(id, datetime.datetime.now())

    dataname = []
    if id.startswith("sodk"):
        dataname = sodk_dataname
    if id.startswith("cam"):
        dataname = cam_dataname

    for key in js:
        if (key != "id" and key != "Acc" and key != "Mag"):
            if id.startswith("sodk"):
                if key not in sodk_dataname:
                    sodk_dataname.append(key)
                    dataname = sodk_dataname
            else:
                if key not in cam_dataname:
                    cam_dataname.append(key)
                    dataname = cam_dataname

    try:
        with open(csvFilename, 'r', newline='') as csvfile:
            n = csvfile.read(1)
    except:
        with open(csvFilename, 'w', newline='') as csvfile:
            csvfile.write(";".join(dataname) + "\n")

    try:
        with open(csvFilename, 'a', newline='') as csvfile:
            data = []
            for key in dataname:
                if key in js:
                    data.append(js[key])
                elif key == "AccX":
                    if "Acc" in js:
                        data.extend(js["Acc"])
                elif key == "MagX":
                    if "Mag" in js:
                        data.extend(js["Mag"])
                elif key not in ["AccY", "AccZ", "MagY", "MagZ"]:
                    data.append("")
            csvfile.write(";".join(map(str, data)) + "\n")

    except Exception as e:
        logging.error(e)


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

    try:
        with open(file, 'r') as file:
            # read lines from the file
            lines = file.readlines()
    except Exception as e:
        logging.error(e)
        return

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

    IntouchFilename = '{}{} {:%Y-%m-%d_%H.%M.%S.%f}.csv'.format(FASTLOADDIR, id, datetime.datetime.now())
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
                    f.write("TK{}_{},0,{},0,{},192\n".format(name.encode(encoding="latin-1", errors="ignore").decode(), key, time_and_date, js[key]))

    except Exception as e:
        # print('File error! {}'.format(e))
        logging.error(e)


def check_message(id):
    fn = 'message_{}.txt'.format(str(id))
    try:
        with open(fn, 'r') as f:
            jsm = json.load(f)
            return json.dumps(jsm)
    except Exception as e:
        logging.error(e)
        return ""


def add_opc_node_cam(num, idx, opcobjects):
    name = "cam" + str(num)
    obj = opcobjects.add_object(idx, name)
    obj.add_property(idx, "idn", num)
    obj.add_property(idx, "id", name)
    for key in cam_dataname:
        if key in ["id", "dt", "Flags"]:
            obj.add_variable(idx, key, None, ua.VariantType.String)
        elif key in ["num", "TAC", "CI"]:
            obj.add_variable(idx, key, 0, ua.VariantType.UInt32)
        else:
            obj.add_variable(idx, key, 0.0, ua.VariantType.Float)
    return obj


def add_opc_node_sodk(num, idx, opcobjects):
    rootname = "sodk" + str(num)
    obj = opcobjects.add_object(idx, rootname)
    obj.add_property(idx, "id", num)
    obj.add_property(idx, "ids", rootname)
    for key in sodk_common_dataname:
        if key in ["id", "dt", "Flags"]:
            obj.add_variable(idx, key, None, ua.VariantType.String)
        elif key in ["num", "TAC", "CI", "time"]:
            obj.add_variable(idx, key, 0, ua.VariantType.UInt32)
        else:
            obj.add_variable(idx, key, 0.0, ua.VariantType.Float)
    for chan in [1, 2, 3, 4, 5, 6, 7, 8]:
        name = rootname + '.' + str(chan)
        objchan = obj.add_object(idx, str(chan))
        objchan.add_property(idx, "ids", name)
        objchan.add_property(idx, "channel", float(num + chan / 10.0))
        for key in sodk_channel_dataname:
            if key in ["id", "dt", "Flags"]:
                objchan.add_variable(idx, key, None, ua.VariantType.String)
            elif key in ["num", "TAC", "CI", "time"]:
                objchan.add_variable(idx, key, 0, ua.VariantType.UInt32)
            else:
                objchan.add_variable(idx, key, 0.0, ua.VariantType.Float)
    return obj


if __name__ == '__main__':

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)-5.5s] %(message)s",
        handlers=[
            logging.FileHandler("debug.log"),
            logging.StreamHandler(sys.stdout)
        ]
    )

    logger = logging.getLogger("opcua")
    logger.setLevel(logging.ERROR)

    # setup our server
    opcserver = Server()
    opcserver.set_endpoint("opc.tcp://" + HOST + ":4840/freeopcua/server/")

    # setup our own namespace, not really necessary but should as spec
    uri = "http://examples.freeopcua.github.io"
    idx = opcserver.register_namespace(uri)

    opcobjects = opcserver.get_objects_node()

    for i in OPCNODES_CAM:
        add_opc_node_cam(i, idx, opcobjects)

    for i in OPCNODES_SODK:
        add_opc_node_sodk(i, idx, opcobjects)

    opcserver.start()

    # Откуда и куда записывать информацию
    outputs = []
    xinputs = []

    # Outgoing message queues (socket:Queue)
    message_queues = {}

    # Создаем серверный сокет без блокирования основного потока в ожидании подключения
    server_socket_tcp = get_non_blocking_server_socket(socket.SOCK_STREAM)
    server_socket_udp = get_non_blocking_server_socket(socket.SOCK_DGRAM)
    inputs = [server_socket_tcp, server_socket_udp]
    # print("TCP/UDP server is running")
    logging.info("TCP/UDP server is running")
    logging.debug("New: {}".format(str(server_socket_tcp)))
    logging.debug("New: {}".format(str(server_socket_udp)))

    while inputs:
        readables, writables, exceptional = select.select(inputs, outputs, xinputs)
        message = b''
        client_address = ('', 0)
        # print("readadle len: " + str(len(readables)))
        for s in readables:
            # print("protocol: " + str(s.proto))
            print("readadle: " + str(s))
            if s == server_socket_tcp:
                connection, client_address = s.accept()
                connection.setblocking(False)
                inputs.append(connection)
                logging.debug("New: {}".format(str(s)))
                continue
            else:  # UDP or connection
                # print("readadle: " + str(s))
                logging.info("Msg: {}".format(str(s)))
                try:
                    (message, client_address) = s.recvfrom(DATASIZE)
                    # print("readadle: " + str(s.type))
                    if s.type == socket.SOCK_STREAM:
                        client_address = s.getpeername()

                    # reply from file message.txt
                    for js in parse_msg(message.decode(encoding="latin-1", errors="ignore")):
                        if "id" in js:
                            msg = check_message(js["id"])
                            if msg:
                                if s not in outputs:
                                    message_queues[s] = queue.Queue()
                                    message_queues[s].put(dict([("message", msg), ("client", client_address)]))
                                    outputs.append(s)

                except Exception as e:
                    # print(e)
                    if s in message_queues:
                        del message_queues[s]
                    logging.error(e)

            if message:
                # Вывод полученных данных на консоль
                # print("{}: {}".format(str(client_address), str(message)))
                logging.info("{}: {}".format(str(client_address), str(message)))
                try:
                    for js in parse_msg(message.decode(encoding="latin-1", errors="ignore")):
                        if "id" in js:
                            id = str(js["id"])
                            rootstring = id.split('.')
                            rootpath = ["{}:{}".format(idx, str(rootstring[0]))]
                            node = ()
                            try:
                                node = opcobjects.get_child(rootpath)
                            except:
                                if id.startswith("cam"):
                                    node = add_opc_node_cam(str(rootstring[0])[3:], idx, opcobjects)
                                if id.startswith("sodk"):
                                    node = add_opc_node_sodk(str(rootstring[0])[4:], idx, opcobjects)

                            for key in js:
                                try:
                                    if key in ["Acc", "Mag"]:
                                        # nodesdata[id][key + 'X'].set_value(js[key][0])
                                        # nodesdata[id][key + 'Y'].set_value(js[key][1])
                                        # nodesdata[id][key + 'Z'].set_value(js[key][2])
                                        var = node.get_child(["{}:{}".format(idx, str(key) + 'X')])
                                        if var:
                                            var.set_value(js[key][0])
                                        var = node.get_child(["{}:{}".format(idx, str(key) + 'Y')])
                                        if var:
                                            var.set_value(js[key][1])
                                        var = node.get_child(["{}:{}".format(idx, str(key) + 'Z')])
                                        if var:
                                            var.set_value(js[key][2])
                                    elif key not in ["id"]:
                                        # nodesdata[id][key].set_value(js[key])
                                        # sodk
                                        if key in sodk_common_dataname + cam_dataname:
                                            var = node.get_child(["{}:{}".format(idx, str(key))])
                                            if var:
                                                var.set_value(js[key])
                                        if len(rootstring) > 1 and key in sodk_channel_dataname:
                                            var = node.get_child(["{}:{}".format(idx, str(rootstring[1])), "{}:{}".format(idx, str(key))])
                                            if var:
                                                var.set_value(js[key])

                                except Exception as e:
                                    logging.error("Parce data:" + e)
                            write_csv(js)
                            write_SQL(js)
                            # write_historian(js)

                except Exception as e:
                    # print(e)
                    logging.error(e)
            else:
                print("close: " + str(s))
                if s not in [server_socket_tcp, server_socket_udp]:
                    inputs.remove(s)
                logging.debug("Cls: {}".format(str(s)))
                s.close()

        for s in writables:
            try:
                msg = message_queues[s].get_nowait()
            except queue.Empty:
                # No messages waiting so stop checking
                # for writability.
                outputs.remove(s)
            else:
                logging.info("Send {} to {}".format(str(msg["message"]), str(msg["client"])))
                if s.type == socket.SOCK_STREAM:
                    s.send(bytes(msg["message"], "latin-1"))
                else:
                    # UDP
                    s.sendto(bytes(msg["message"], "latin-1"), msg["client"])

        for s in exceptional:
            logging.info('exception condition on {}'.format(str(s.getpeername())))
            # Stop listening for input on the connection
            inputs.remove(s)
            if s in outputs:
                outputs.remove(s)
            s.close()
            # Remove message queue
            if s in message_queues:
                del message_queues[s]
